/**
 * .cvoc binary format (little-endian; this project only ever runs on x86_64/aarch64 hosts, both
 * little-endian, so no byte-swap path exists - see Load()/ConvertTextToCompact()):
 *
 *   header (64 bytes, packed):
 *     char     magic[8]            "CVOC0001"
 *     uint32_t k                   branching factor
 *     uint32_t l                   depth levels
 *     uint32_t scoring_type        DBoW2::ScoringType (must be L1_NORM=0)
 *     uint32_t weighting_type      DBoW2::WeightingType (must be TF_IDF=0)
 *     uint32_t n_nodes             number of tree nodes, INCLUDING the root (index 0)
 *     uint32_t n_words             number of leaf (word) nodes
 *     uint32_t total_children      sum of every node's child count == n_nodes - 1
 *     uint64_t payload_crc64       CRC64/XZ over every byte following this header
 *     uint8_t  reserved[20]        zero, pads the header to 64 bytes
 *
 *   payload (tightly packed, no gaps - every array below starts at an offset that is already a
 *   multiple of its element size, since the header is 64 bytes and n_nodes*32 is always a
 *   multiple of 4; see the alignment argument in task-P2-report.md):
 *     double   weight[n_nodes]
 *     uint32_t child_first[n_nodes]   offset into child_idx[] of node i's first child
 *     uint32_t child_count[n_nodes]   number of children of node i (0 => leaf)
 *     uint32_t word_id[n_nodes]       meaningful only where child_count[i] == 0
 *     uint8_t  desc[n_nodes][32]      raw ORB descriptor bytes (root's entry is unused, zeroed)
 *     uint32_t child_idx[total_children]   flattened children, PER-PARENT PUSH_BACK ORDER
 *                                           preserved exactly (this is what makes transform()'s
 *                                           strict '<' child-descent bit-identical to the text
 *                                           path - see TemplatedVocabulary.h:1240-1249).
 *
 * NodeId == array index == text-file insertion order, exactly matching what DBoW2's own
 * loadFromTextFile() assigns (TemplatedVocabulary.h:1385-1387). Renumbering nodes here would
 * silently change FeatureVector's NodeId keys and therefore SearchByBoW's iteration order -
 * ConvertTextToCompact() below copies DBoW2's own parsed tree verbatim rather than
 * re-deriving it, specifically to avoid that class of bug.
 */

#include "CompactVocabulary.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <iostream>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h"

namespace ORB_SLAM3
{

namespace
{

using StockVocabulary = DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>;

// Re-exposes the protected parse result of TemplatedVocabulary::loadFromTextFile() (m_nodes,
// m_words, and the k/L/scoring/weighting fields) so ConvertTextToCompact() can serialize DBoW2's
// OWN in-memory tree verbatim, instead of re-implementing the (word_id-order-dependent, easy to
// get subtly wrong) text grammar independently. This is the crux of the bit-exactness guarantee:
// every byte written to the .cvoc file is copied from the exact structure DBoW2's real parser
// produced, not re-derived.
class TextVocabularyIntrospector : public StockVocabulary
{
public:
    using StockVocabulary::Node;
    using StockVocabulary::m_k;
    using StockVocabulary::m_L;
    using StockVocabulary::m_scoring;
    using StockVocabulary::m_weighting;
    using StockVocabulary::m_nodes;
    using StockVocabulary::m_words;
};

#pragma pack(push, 1)
struct CvocHeader
{
    char magic[8];
    std::uint32_t k;
    std::uint32_t l;
    std::uint32_t scoring_type;
    std::uint32_t weighting_type;
    std::uint32_t n_nodes;
    std::uint32_t n_words;
    std::uint32_t total_children;
    std::uint64_t payload_crc64;
    std::uint8_t reserved[20];
};
#pragma pack(pop)
static_assert(sizeof(CvocHeader) == 64U, "CvocHeader must be exactly 64 bytes (.cvoc format)");

constexpr char kCvocMagic[8] = {'C', 'V', 'O', 'C', '0', '0', '0', '1'};

// Reflected CRC-64/XZ (poly 0xC96C5795D7870F42, init/xorout all-ones) - a well-known, simple,
// table-accelerated algorithm. This is purely an internal corruption/truncation check between
// ConvertTextToCompact() and Load(); it is never compared against any external CRC64 value, so
// any well-defined 64-bit checksum would do, but using a standard one avoids reinventing one
// from scratch.
std::uint64_t Crc64Table(const unsigned int byte_value)
{
    std::uint64_t crc = byte_value;
    for (int bit = 0; bit < 8; ++bit)
    {
        crc = (crc & 1ULL) != 0ULL ? (crc >> 1) ^ 0xC96C5795D7870F42ULL : (crc >> 1);
    }
    return crc;
}

std::uint64_t ComputeCrc64(const unsigned char* const data, const std::size_t length)
{
    static const std::array<std::uint64_t, 256> table = []() {
        std::array<std::uint64_t, 256> generated{};
        for (unsigned int byte_value = 0; byte_value < 256U; ++byte_value)
        {
            generated[byte_value] = Crc64Table(byte_value);
        }
        return generated;
    }();

    std::uint64_t crc = 0xFFFFFFFFFFFFFFFFULL;
    for (std::size_t index = 0; index < length; ++index)
    {
        crc = table[(crc ^ data[index]) & 0xFFULL] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFFFFFFFFFFULL;
}

// Shared by ConvertTextToCompact() (sizing the write buffer) and Load() (validating the file
// length before trusting the header's counts) so the two can never disagree about the layout.
std::size_t ComputePayloadBytes(const std::size_t n_nodes, const std::size_t total_children)
{
    return n_nodes * sizeof(double) +            // weight
           n_nodes * sizeof(std::uint32_t) +      // child_first
           n_nodes * sizeof(std::uint32_t) +      // child_count
           n_nodes * sizeof(std::uint32_t) +      // word_id
           n_nodes * 32U +                        // desc
           total_children * sizeof(std::uint32_t);  // child_idx
}

// Retry-on-EINTR/partial-write loop, mirroring MemoryGovernor.cc's WriteAll idiom (this is a
// one-time/rare conversion path, not a per-frame hot path, but there is no reason to leave a
// short write unhandled).
bool WriteAll(const int fd, const void* const data, const std::size_t length)
{
    const unsigned char* cursor = static_cast<const unsigned char*>(data);
    std::size_t remaining = length;
    while (remaining > 0U)
    {
        const ssize_t written = write(fd, cursor, remaining);
        if (written < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            return false;
        }
        cursor += written;
        remaining -= static_cast<std::size_t>(written);
    }
    return true;
}

// Builds the payload byte buffer (everything after the header) from an already-parsed
// introspector, and fills in the header's count/type fields (payload_crc64 is filled in by the
// caller once this buffer is complete, since the CRC covers exactly this buffer).
std::vector<unsigned char> BuildPayload(const TextVocabularyIntrospector& introspector,
                                         CvocHeader& header_out)
{
    const std::size_t n_nodes = introspector.m_nodes.size();
    std::size_t total_children = 0;
    for (const auto& node : introspector.m_nodes)
    {
        total_children += node.children.size();
    }

    std::vector<unsigned char> payload(ComputePayloadBytes(n_nodes, total_children));
    unsigned char* cursor = payload.data();
    double* const weight_out = reinterpret_cast<double*>(cursor);
    cursor += n_nodes * sizeof(double);
    std::uint32_t* const child_first_out = reinterpret_cast<std::uint32_t*>(cursor);
    cursor += n_nodes * sizeof(std::uint32_t);
    std::uint32_t* const child_count_out = reinterpret_cast<std::uint32_t*>(cursor);
    cursor += n_nodes * sizeof(std::uint32_t);
    std::uint32_t* const word_id_out = reinterpret_cast<std::uint32_t*>(cursor);
    cursor += n_nodes * sizeof(std::uint32_t);
    unsigned char* const desc_out = cursor;
    cursor += n_nodes * 32U;
    std::uint32_t* const child_idx_out = reinterpret_cast<std::uint32_t*>(cursor);

    std::uint32_t running_child_offset = 0;
    for (std::size_t node_id = 0; node_id < n_nodes; ++node_id)
    {
        const auto& node = introspector.m_nodes[node_id];
        weight_out[node_id] = node.weight;
        word_id_out[node_id] = node.word_id;
        child_first_out[node_id] = running_child_offset;
        child_count_out[node_id] = static_cast<std::uint32_t>(node.children.size());
        for (std::size_t child_index = 0; child_index < node.children.size(); ++child_index)
        {
            child_idx_out[running_child_offset + child_index] =
                static_cast<std::uint32_t>(node.children[child_index]);
        }
        running_child_offset += static_cast<std::uint32_t>(node.children.size());

        unsigned char* const desc_row = desc_out + node_id * 32U;
        if (node.descriptor.empty())
        {
            std::fill(desc_row, desc_row + 32U, static_cast<unsigned char>(0));  // root: never read
        }
        else
        {
            std::memcpy(desc_row, node.descriptor.data, 32U);
        }
    }

    header_out = CvocHeader{};
    std::memcpy(header_out.magic, kCvocMagic, sizeof(kCvocMagic));
    header_out.k = static_cast<std::uint32_t>(introspector.m_k);
    header_out.l = static_cast<std::uint32_t>(introspector.m_L);
    header_out.scoring_type = static_cast<std::uint32_t>(introspector.m_scoring);
    header_out.weighting_type = static_cast<std::uint32_t>(introspector.m_weighting);
    header_out.n_nodes = static_cast<std::uint32_t>(n_nodes);
    header_out.n_words = static_cast<std::uint32_t>(introspector.m_words.size());
    header_out.total_children = static_cast<std::uint32_t>(total_children);
    return payload;
}

}  // namespace

bool CompactVocabulary::ConvertTextToCompact(const std::string& text_vocabulary_path,
                                              const std::string& compact_vocabulary_path)
{
    TextVocabularyIntrospector introspector;
    if (!introspector.loadFromTextFile(text_vocabulary_path))
    {
        std::cerr << "CompactVocabulary::ConvertTextToCompact: failed to parse "
                  << text_vocabulary_path << std::endl;
        return false;
    }
    if (introspector.m_scoring != DBoW2::L1_NORM || introspector.m_weighting != DBoW2::TF_IDF)
    {
        std::cerr << "CompactVocabulary::ConvertTextToCompact: " << text_vocabulary_path
                  << " uses scoring=" << introspector.m_scoring
                  << " weighting=" << introspector.m_weighting
                  << " - this format only supports L1_NORM(0)/TF_IDF(0)" << std::endl;
        return false;
    }

    CvocHeader header{};
    const std::vector<unsigned char> payload = BuildPayload(introspector, header);
    header.payload_crc64 = ComputeCrc64(payload.data(), payload.size());

    const std::string temp_path =
        compact_vocabulary_path + ".tmp." + std::to_string(static_cast<long>(getpid()));
    const int fd = open(temp_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0)
    {
        std::cerr << "CompactVocabulary::ConvertTextToCompact: cannot create " << temp_path
                  << std::endl;
        return false;
    }
    const bool write_ok =
        WriteAll(fd, &header, sizeof(header)) && WriteAll(fd, payload.data(), payload.size());
    close(fd);
    if (!write_ok)
    {
        std::cerr << "CompactVocabulary::ConvertTextToCompact: write failed for " << temp_path
                  << std::endl;
        std::remove(temp_path.c_str());
        return false;
    }
    if (std::rename(temp_path.c_str(), compact_vocabulary_path.c_str()) != 0)
    {
        std::cerr << "CompactVocabulary::ConvertTextToCompact: rename " << temp_path << " -> "
                  << compact_vocabulary_path << " failed" << std::endl;
        std::remove(temp_path.c_str());
        return false;
    }
    return true;
}

bool CompactVocabulary::Load(const std::string& compact_vocabulary_path)
{
    Unmap();
    const int fd = open(compact_vocabulary_path.c_str(), O_RDONLY);
    if (fd < 0)
    {
        std::cerr << "CompactVocabulary::Load: cannot open " << compact_vocabulary_path
                  << std::endl;
        return false;
    }
    struct stat file_stat{};
    if (fstat(fd, &file_stat) != 0 || file_stat.st_size < static_cast<off_t>(sizeof(CvocHeader)))
    {
        std::cerr << "CompactVocabulary::Load: " << compact_vocabulary_path
                  << " missing or smaller than the header" << std::endl;
        close(fd);
        return false;
    }
    const std::size_t file_length = static_cast<std::size_t>(file_stat.st_size);
    void* const base = mmap(nullptr, file_length, PROT_READ, MAP_SHARED, fd, 0);
    close(fd);  // the mapping keeps the file referenced; the fd itself is no longer needed
    if (base == MAP_FAILED)
    {
        std::cerr << "CompactVocabulary::Load: mmap failed for " << compact_vocabulary_path
                  << std::endl;
        return false;
    }

    const CvocHeader* const header = static_cast<const CvocHeader*>(base);
    if (std::memcmp(header->magic, kCvocMagic, sizeof(kCvocMagic)) != 0)
    {
        std::cerr << "CompactVocabulary::Load: bad magic in " << compact_vocabulary_path
                  << std::endl;
        munmap(base, file_length);
        return false;
    }
    if (header->scoring_type != static_cast<std::uint32_t>(DBoW2::L1_NORM) ||
        header->weighting_type != static_cast<std::uint32_t>(DBoW2::TF_IDF) ||
        header->n_nodes == 0U)
    {
        std::cerr << "CompactVocabulary::Load: unsupported scoring/weighting type or empty tree in "
                  << compact_vocabulary_path << std::endl;
        munmap(base, file_length);
        return false;
    }
    const std::size_t expected_payload = ComputePayloadBytes(header->n_nodes, header->total_children);
    if (file_length != sizeof(CvocHeader) + expected_payload)
    {
        std::cerr << "CompactVocabulary::Load: size mismatch in " << compact_vocabulary_path
                  << std::endl;
        munmap(base, file_length);
        return false;
    }
    const unsigned char* const payload = static_cast<const unsigned char*>(base) + sizeof(CvocHeader);
    if (ComputeCrc64(payload, expected_payload) != header->payload_crc64)
    {
        std::cerr << "CompactVocabulary::Load: CRC64 mismatch in " << compact_vocabulary_path
                  << std::endl;
        munmap(base, file_length);
        return false;
    }

    mMappedBase = base;
    mMappedLength = file_length;
    mBranchingFactor = header->k;
    mDepthLevels = header->l;
    mNumNodes = header->n_nodes;
    mNumWords = header->n_words;

    const unsigned char* cursor = payload;
    mWeight = reinterpret_cast<const double*>(cursor);
    cursor += static_cast<std::size_t>(mNumNodes) * sizeof(double);
    mChildFirst = reinterpret_cast<const std::uint32_t*>(cursor);
    cursor += static_cast<std::size_t>(mNumNodes) * sizeof(std::uint32_t);
    mChildCount = reinterpret_cast<const std::uint32_t*>(cursor);
    cursor += static_cast<std::size_t>(mNumNodes) * sizeof(std::uint32_t);
    mWordId = reinterpret_cast<const std::uint32_t*>(cursor);
    cursor += static_cast<std::size_t>(mNumNodes) * sizeof(std::uint32_t);
    mDescriptorBytes = cursor;
    cursor += static_cast<std::size_t>(mNumNodes) * 32U;
    mChildIdx = reinterpret_cast<const std::uint32_t*>(cursor);
    return true;
}

void CompactVocabulary::Unmap()
{
    if (mMappedBase != nullptr)
    {
        munmap(mMappedBase, mMappedLength);
    }
    mMappedBase = nullptr;
    mMappedLength = 0;
    mWeight = nullptr;
    mChildFirst = nullptr;
    mChildCount = nullptr;
    mWordId = nullptr;
    mDescriptorBytes = nullptr;
    mChildIdx = nullptr;
}

CompactVocabulary::~CompactVocabulary()
{
    Unmap();
}

cv::Mat CompactVocabulary::DescriptorAt(const std::uint32_t node_id) const
{
    // Non-owning view directly into the mmap'd region: the cv::Mat(rows, cols, type, data)
    // constructor never allocates/copies/refcounts, so this is a zero-cost cast, not a fault.
    return cv::Mat(1, 32, CV_8U,
                   const_cast<unsigned char*>(mDescriptorBytes + static_cast<std::size_t>(node_id) * 32U));
}

// Bit-exact port of TemplatedVocabulary::transform(feature, word_id, weight, nid, levelsup)
// (TemplatedVocabulary.h:1218-1259): same strict '<' child selection (ties keep the
// lower-indexed, i.e. earlier-inserted, child - this is exactly why child_idx[] must preserve
// push_back order), same nid-capture-at-level logic, same leaf/word_id/weight readout.
void CompactVocabulary::TransformOne(const DBoW2::FORB::TDescriptor& feature, DBoW2::WordId& word_id,
                                      DBoW2::WordValue& weight, DBoW2::NodeId* const node_id_out,
                                      const int levelsup) const
{
    const int nid_level = static_cast<int>(mDepthLevels) - levelsup;
    std::uint32_t final_id = 0;  // root
    if (nid_level <= 0 && node_id_out != nullptr)
    {
        *node_id_out = 0;
    }

    int current_level = 0;
    do
    {
        ++current_level;
        const std::uint32_t first_child = mChildFirst[final_id];
        const std::uint32_t num_children = mChildCount[final_id];
        std::uint32_t best_child = mChildIdx[first_child];
        double best_distance = static_cast<double>(DBoW2::FORB::distance(feature, DescriptorAt(best_child)));
        for (std::uint32_t child_index = 1; child_index < num_children; ++child_index)
        {
            const std::uint32_t candidate = mChildIdx[first_child + child_index];
            const double distance =
                static_cast<double>(DBoW2::FORB::distance(feature, DescriptorAt(candidate)));
            if (distance < best_distance)
            {
                best_distance = distance;
                best_child = candidate;
            }
        }
        final_id = best_child;
        if (node_id_out != nullptr && current_level == nid_level)
        {
            *node_id_out = final_id;
        }
    } while (mChildCount[final_id] != 0U);  // stop at a leaf (isLeaf() == children.empty())

    word_id = mWordId[final_id];
    weight = mWeight[final_id];
}

void CompactVocabulary::Transform(const std::vector<DBoW2::FORB::TDescriptor>& features,
                                  DBoW2::BowVector& bow_vector, DBoW2::FeatureVector& feature_vector,
                                  const int levelsup) const
{
    bow_vector.clear();
    feature_vector.clear();
    if (mNumWords == 0U)  // matches TemplatedVocabulary::transform's `if(empty()) return;`
    {
        return;
    }

    // Load() asserts weighting==TF_IDF/scoring==L1_NORM, so only the TF_IDF branch of
    // TemplatedVocabulary::transform (TemplatedVocabulary.h:1145-1162) is reachable, and
    // L1Scoring::mustNormalize() always returns true - so the stock code's IDF/BINARY branch and
    // its "!must" per-feature averaging branch are dead code for this vocabulary and are
    // intentionally not reproduced here.
    unsigned int feature_index = 0;
    for (const auto& feature : features)
    {
        DBoW2::WordId word_id = 0;
        DBoW2::NodeId node_id = 0;
        DBoW2::WordValue weight = 0.0;
        TransformOne(feature, word_id, weight, &node_id, levelsup);
        if (weight > 0.0)
        {
            bow_vector.addWeight(word_id, weight);
            feature_vector.addFeature(node_id, feature_index);
        }
        ++feature_index;
    }

    DBoW2::LNorm norm_type = DBoW2::L1;
    if (mScoringObject.mustNormalize(norm_type))
    {
        bow_vector.normalize(norm_type);
    }
}

double CompactVocabulary::Score(const DBoW2::BowVector& bow_vector_a,
                                 const DBoW2::BowVector& bow_vector_b) const
{
    return mScoringObject.score(bow_vector_a, bow_vector_b);
}

}  // namespace ORB_SLAM3
