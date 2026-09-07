/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file javad_greis.h
 *
 * JAVAD GREIS/JPS receiver support.
 */

#pragma once

#include "gps_helper.h"

class GPSDriverJavadGreis : public GPSHelper
{
public:
	GPSDriverJavadGreis(GPSCallbackPtr callback, void *callback_user, sensor_gps_s *gps_position);
	~GPSDriverJavadGreis() override = default;

	int configure(unsigned &baudrate, const GPSConfig &config) override;
	int receive(unsigned timeout) override;
	bool receiverReady() const override;

private:
	enum class ParseState : uint8_t {
		Id1,
		Id2,
		Length1,
		Length2,
		Length3,
		Body,
	};

	enum class Message : uint8_t {
		Unknown,
		GT,
		ST,
		RT,
		ET,
		PG,
		VG,
		PV,
		SG,
		DP,
		PS,
	};

	enum class ConfigState : uint8_t {
		DisableOutput,
		EnableStream,
		Complete,
	};

	struct MessageInfo {
		Message message;
		uint16_t minimum_length;
		uint8_t checksum_length;
	};

	struct TimeMark {
		bool valid{false};
		uint16_t week{0};
		uint32_t tow_ms{0};
	};

	struct Vector3d {
		double x{0.0};
		double y{0.0};
		double z{0.0};
	};

	struct Vector3f {
		float x{0.0f};
		float y{0.0f};
		float z{0.0f};
	};

	struct Epoch {
		bool valid{false};
		bool published{false};

		bool have_rt{false};
		bool have_st{false};
		bool have_et{false};
		bool have_gt{false};
		bool have_position{false};
		bool have_velocity{false};
		bool have_pv{false};
		bool have_pv_geodetic{false};
		bool have_sg{false};
		bool have_dp{false};
		bool have_ps{false};

		uint32_t rt_time_ms{0};
		uint32_t st_time_ms{0};
		uint32_t et_time_ms{0};

		uint16_t week{0};
		uint32_t tow_ms{0};

		double latitude_deg{0.0};
		double longitude_deg{0.0};
		double ellipsoid_height_m{0.0};
		float position_sigma_m{-1.0f};
		uint8_t position_sol_type{0};

		Vector3f velocity_ned{};
		float velocity_sigma_m_s{-1.0f};
		uint8_t velocity_sol_type{0};

		Vector3d pv_position_ecef{};
		Vector3f pv_velocity_ecef{};
		double pv_latitude_deg{0.0};
		double pv_longitude_deg{0.0};
		double pv_ellipsoid_height_m{0.0};
		Vector3f pv_velocity_ned{};
		float pv_position_sigma_m{-1.0f};
		float pv_velocity_sigma_m_s{-1.0f};
		uint8_t pv_sol_type{0};

		float horizontal_accuracy_m{0.0f};
		float vertical_accuracy_m{0.0f};
		float speed_accuracy_m_s{0.0f};
		uint8_t sg_sol_type{0};

		float hdop{99.9f};
		float vdop{99.9f};
		uint8_t dp_sol_type{0};

		uint8_t satellites_used{0};
		uint8_t ps_sol_type{0};
		uint8_t st_sol_type{0};
	};

	enum SeenMessages : uint32_t {
		SeenGT = 1 << 0,
		SeenRT = 1 << 1,
		SeenST = 1 << 2,
		SeenET = 1 << 3,
		SeenPG = 1 << 4,
		SeenVG = 1 << 5,
		SeenPV = 1 << 6,
		SeenSG = 1 << 7,
		SeenDP = 1 << 8,
		SeenPS = 1 << 9,
	};

	bool parseByte(uint8_t byte);
	bool dispatchMessage();

	void resetParser();
	void restartParser(uint8_t byte);
	void resetEpoch();

	static bool isMessageIdByte(uint8_t byte);
	static int8_t asciiHexValue(uint8_t byte);
	static uint8_t checksumRotate(uint8_t value);
	void startChecksum();
	void addChecksumByte(uint8_t byte);
	bool checksumValid() const;

	MessageInfo messageInfo(const char id[2]) const;
	uint16_t payloadDataLength() const;

	bool decodeGT();
	bool decodeRT();
	bool decodeST();
	bool decodeET();
	bool decodePG();
	bool decodeVG();
	bool decodePV();
	bool decodeSG();
	bool decodeDP();
	bool decodePS();

	bool epochAcceptsData() const;
	bool epochComplete() const;
	bool publishEpoch();
	void markSeen(uint32_t bit);
	bool requiredMessagesSeen() const;

	void updateConfiguration(bool force = false);
	bool sendCommand(const char *command);

	uint16_t readU2(uint16_t offset) const;
	uint32_t readU4(uint16_t offset) const;
	float readFloat(uint16_t offset) const;
	double readDouble(uint16_t offset) const;

	static uint8_t solutionTypeToFix(uint8_t solution_type);
	static bool ecefVelocityToNed(const Vector3f &ecef_velocity, double latitude_rad, double longitude_rad,
				      Vector3f &ned_velocity);

	static constexpr uint16_t MaxPayloadLength = 0x0fff;
	static constexpr uint16_t PayloadBufferLength = 128;

	static constexpr uint16_t GTMinPayloadLength = 8;
	static constexpr uint16_t STMinPayloadLength = 6;
	static constexpr uint16_t RTMinPayloadLength = 5;
	static constexpr uint16_t ETMinPayloadLength = 5;
	static constexpr uint16_t PGMinPayloadLength = 30;
	static constexpr uint16_t VGMinPayloadLength = 18;
	static constexpr uint16_t PVMinPayloadLength = 46;
	static constexpr uint16_t SGMinPayloadLength = 18;
	static constexpr uint16_t DPMinPayloadLength = 18;
	static constexpr uint16_t PSMinPayloadLength = 9;

	static constexpr uint64_t InitialConfigDelayUs = 200000;
	static constexpr uint64_t ConfigRetryIntervalUs = 3000000;
	static constexpr uint8_t PSKnownExtraSystems = 5;

	sensor_gps_s *_gps_position{nullptr};

	ParseState _parse_state{ParseState::Id1};
	Message _message{Message::Unknown};
	ConfigState _config_state{ConfigState::DisableOutput};

	char _id[2]{};
	char _length_ascii[3]{};
	uint16_t _payload_length{0};
	uint16_t _payload_bytes_read{0};
	uint16_t _payload_bytes_stored{0};
	uint8_t _payload[PayloadBufferLength]{};
	uint8_t _checksum{0};
	uint8_t _checksum_bytes[2]{};
	uint8_t _checksum_length{0};
	uint16_t _minimum_payload_length{0};

	uint32_t _seen_messages{0};
	uint64_t _next_config_time_us{0};

	TimeMark _pending_gt{};
	Epoch _epoch{};
};
