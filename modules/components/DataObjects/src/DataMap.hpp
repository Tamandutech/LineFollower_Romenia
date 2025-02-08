#ifndef DATA_MAP_HPP
#define DATA_MAP_HPP

#include <cxxabi.h>

#include <atomic>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <list>
#include <memory>
#include <sstream>
#include <string>
#include <typeinfo>
#include <vector>

#include "DataManager.hpp"
#include "DataStorage.hpp"
#include "IDataAbstract.hpp"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "../../RobotData/src/dataEnums.h"

struct MapData {
  uint32_t MapTime;
  int32_t MapEncMedia;

  /**
   * @brief Ajuste da posição da marcação em pulsos.
   */
  int16_t MapOffset;

  /**
   * @brief Status do trecho atual.
   */
  TrackSegment MapTrackStatus;

  /**
   * @brief Distância que será percorrida até o robô atingir a velocidade máxima no trecho atual.
   */
  float MapAccelerationSpace;

  /**
   * @brief Distância que será percorrida até o robô atingir a velocidade da próxima curva.
   */
  float MapDecelerationSpace;
};

class DataMap : public IDataAbstract {
 public:
  DataMap(std::string name, std::string parentObjectName);
  virtual ~DataMap();

  MapData getData(uint8_t posicao);
  void setData(uint8_t posicao, MapData mapData);

  void newData(MapData mapData);
  void newData(std::string mapData);

  /// @brief Definir dado estruturado de mapeamento
  /// @param mapData Dados da struct separados por ",". Ex.:
  /// "Posicao,MapTime,MapEncMedia,MapEncLeft,MapEncRight,MapStatus"
  void setData(std::string data);

  /// @brief Obter o primeiro dado da lista em formato string
  /// @return Dados da struct separados por ",". Ex.:
  /// "Posicao,MapTime,MapEncMedia,MapEncLeft,MapEncRight,MapStatus"
  std::string getDataString();

  /// @brief Obter dado na posição especificada da lista em formato string
  /// @param ctrl Posição da lista
  /// @return Dados da struct separados por ",". Ex.:
  /// "Posicao,MapTime,MapEncMedia,MapEncLeft,MapEncRight,MapStatus"
  std::string getDataString(std::string ctrl);

  std::uint8_t getSize();

  void saveData();
  void loadData();

  void clearAllData();
  void clearData(uint8_t pos);

  void setStreamInterval(uint32_t interval);
  uint32_t getStreamInterval();
  void setStreamTime(uint32_t streamTime);
  uint32_t getStreamTime();
  uint32_t getLastChange();

 protected:
 private:
  DataStorage *dataStorage;
  DataManager *dataManager;

  std::list<MapData> mapDataList;
  static std::mutex mapDataListMutex;

  std::atomic<uint32_t> time_last_change;  // Tempo de execução em que o dado
                                           // foi alterado pela última vez
  std::atomic<uint32_t>
      stream_interval;  // Intervalo que o stream do atributo será feito
  std::atomic<uint32_t>
      stream_time;  // momento em que o stream do atributo deve ser feito
};

#endif