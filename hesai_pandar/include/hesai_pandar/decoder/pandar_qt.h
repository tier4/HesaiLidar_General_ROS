#pragma once
/**
 * Pandar QT
 */
#include <cstdint>

namespace pandar_qt
{
// Head
constexpr size_t HEAD_SIZE = 12;
constexpr size_t PRE_HEADER_SIZE = 6;
constexpr size_t HEADER_SIZE = 6;
// Body
constexpr size_t BLOCK_NUM = 4;
constexpr size_t BLOCK_HEADER_AZIMUTH = 2;
constexpr size_t UNIT_NUM = 64;
constexpr size_t UNIT_SIZE = 4;
constexpr size_t BLOCK_SIZE = UNIT_SIZE * UNIT_NUM + BLOCK_HEADER_AZIMUTH;
constexpr size_t BODY_SIZE = BLOCK_SIZE * BLOCK_NUM;
//Tail
constexpr size_t RESERVED_SIZE = 10;
constexpr size_t ENGINE_VELOCITY = 2;
constexpr size_t TIMESTAMP_SIZE = 4;
constexpr size_t ECHO_SIZE = 1;
constexpr size_t FACTORY_SIZE = 1;
constexpr size_t UTC_SIZE = 6;
constexpr size_t SEQUENCE_SIZE = 4;
constexpr size_t PACKET_TAIL_SIZE = 28;
constexpr size_t PACKET_TAIL_WITHOUT_UDPSEQ_SIZE = 24;

// All
constexpr size_t PACKET_SIZE = HEAD_SIZE + BODY_SIZE + PACKET_TAIL_SIZE;
constexpr size_t PACKET_WITHOUT_UDPSEQ_SIZE =
  HEAD_SIZE + BODY_SIZE + PACKET_TAIL_WITHOUT_UDPSEQ_SIZE;

struct Header
{
  uint16_t sob;            // 0xFFEE 2bytes
  int8_t chProtocolMajor;  // Protocol Version Major 1byte
  int8_t chProtocolMinor;  // Protocol Version Minor 1byte
  int8_t chLaserNumber;    // laser number 1byte
  int8_t chBlockNumber;    // block number 1byte
  int8_t chReturnType;     // return mode 1 byte  when dual return 0-Single Return
                           // 1-The first block is the 1 st return.
                           // 2-The first block is the 2 nd return
  int8_t chDisUnit;        // Distance unit, 4mm
};

struct Unit
{
  double distance;
  uint16_t intensity;
  uint16_t confidence;
};

struct Block
{
  uint16_t azimuth;  // packet angle,Azimuth = RealAzimuth * 100
  Unit units[UNIT_NUM];
};

struct Packet
{
  Header header;
  Block blocks[BLOCK_NUM];
  uint32_t usec;  // ms
  uint32_t echo;
  tm t;
};
}  // namespace pandar_qt