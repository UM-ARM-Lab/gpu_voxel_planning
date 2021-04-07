#ifndef MEMORIZED_SWEPT_VOLUMES
#define MEMORIZED_SWEPT_VOLUMES

#include <arc_utilities/serialization.hpp>
#include <arc_utilities/zlib_helpers.hpp>
#include <graph_planner/dijkstras_addons.hpp>
#include <map>
#include <utility>

#include "gpu_voxel_planning/maps/prob_map.hpp"

namespace GVP {

class MemorizedSweptVolume : public std::map<arc_dijkstras::HashableEdge, SparseGrid> {
 public:
  uint64_t serializeSelf(std::vector<uint8_t>& buffer) const {
    using namespace arc_utilities;
    using namespace arc_dijkstras;
    // auto key_serializer = [](const HashableEdge& e, std::vector<uint8_t>& buffer)
    //     {
    //         uint64_t bytes_written = SerializeFixedSizePOD(e.first, buffer);
    //         bytes_written += SerializeFixedSizePOD(e.second, buffer);
    //         return bytes_written;
    //     };

    // auto value_serializer = [](const SparseGrid& g, std::vector<uint8_t>& buffer)
    //     {
    //         return g.serializeSelf(buffer);
    //     };
    // return SerializeMap<HashableEdge, SparseGrid>(*this, buffer, key_serializer, value_serializer);

    const uint64_t start_buffer_size = buffer.size();
    const uint64_t size = (uint64_t)this->size();
    SerializeFixedSizePOD(size, buffer);
    typename std::map<HashableEdge, SparseGrid>::const_iterator itr;
    for (itr = this->begin(); itr != this->end(); ++itr) {
      // SerializePair<Key, T>(*itr, buffer, key_serializer, value_serializer);
      SerializeFixedSizePOD(itr->first.first, buffer);
      SerializeFixedSizePOD(itr->first.second, buffer);
      itr->second.serializeSelf(buffer);
    }
    return buffer.size() - start_buffer_size;
  }

  uint64_t deserializeSelf(std::vector<uint8_t>& buffer, uint64_t& buffer_position,
                           uint64_t max_node_index = std::numeric_limits<uint64_t>::max()) {
    using namespace arc_utilities;
    using namespace arc_dijkstras;

    assert(buffer_position < buffer.size());
    const uint64_t buffer_start_position = buffer_position;
    auto result = DeserializeFixedSizePOD<uint64_t>(buffer, buffer_position);
    buffer_position += result.second;
    const uint64_t size = result.first;
    std::cout << "Deserializing " << size << " edges\n";

    for (uint64_t i = 0; i < size; i++) {
      auto res1 = DeserializeFixedSizePOD<uint64_t>(buffer, buffer_position);
      uint64_t n1 = res1.first;
      buffer_position += res1.second;
      auto res2 = DeserializeFixedSizePOD<uint64_t>(buffer, buffer_position);
      uint64_t n2 = res2.first;
      buffer_position += res2.second;

      if (n1 >= max_node_index || n2 >= max_node_index) {
        SparseGrid s;
        s.deserializeSelf(buffer, buffer_position);
        continue;
      }

      HashableEdge e = std::make_pair(n1, n2);
      (*this)[e].deserializeSelf(buffer, buffer_position);
    }
    return buffer_start_position - buffer_position;
  }

  void saveToFile(const std::string& filepath) {
    std::vector<uint8_t> buffer;
    serializeSelf(buffer);
    ZlibHelpers::CompressAndWriteToFile(buffer, filepath);
  }

  void loadFromFile(const std::string& filepath, uint64_t max_node_index = std::numeric_limits<uint64_t>::max()) {
    std::vector<uint8_t> buffer = ZlibHelpers::LoadFromFileAndDecompress(filepath);
    uint64_t start = 0;
    deserializeSelf(buffer, start, max_node_index);
  }
};
}  // namespace GVP

#endif
