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
 * @file javad_greis.cpp
 *
 * JAVAD GREIS/JPS receiver support.
 */

#include "javad_greis.h"

#include <cmath>
#include <cstring>

namespace
{
constexpr unsigned kDefaultBaudrate = 115200;
constexpr double kRadToDeg = 57.295779513082320876798154814105;
constexpr double kDegToRad = 0.017453292519943295769236907684886;
constexpr double kWgs84A = 6378137.0;
constexpr float kUnknownDop = 99.9f;
constexpr float kFallbackCourseVarianceRad = 0.1f;

constexpr const char kDisableOutputCommand[] = "dm\n";
constexpr const char kEnableStreamCommand[] = "dm,;em,,jps/{RT,GT,ST,PG,VG,SG,DP,PS,ET}:0.1\n";
}

GPSDriverJavadGreis::GPSDriverJavadGreis(GPSCallbackPtr callback, void *callback_user, sensor_gps_s *gps_position) :
	GPSHelper(callback, callback_user),
	_gps_position(gps_position)
{
	resetParser();
	resetEpoch();
}

int
GPSDriverJavadGreis::configure(unsigned &baudrate, const GPSConfig &config)
{
	if (config.output_mode == OutputMode::RTCM) {
		GPS_WARN("JAVAD GREIS: RTCM output mode is not supported");
		return -1;
	}

	resetParser();
	resetEpoch();

	_seen_messages = 0;
	_pending_gt = {};
	_config_state = ConfigState::DisableOutput;
	_next_config_time_us = gps_absolute_time();

	if (baudrate == 0) {
		if (setBaudrate(kDefaultBaudrate) != 0) {
			return -1;
		}

		baudrate = kDefaultBaudrate;
	}

	updateConfiguration(true);
	return 0;
}

bool
GPSDriverJavadGreis::receiverReady() const
{
	return _config_state == ConfigState::Complete;
}

int
GPSDriverJavadGreis::receive(unsigned timeout)
{
	uint8_t buffer[GPS_READ_BUFFER_SIZE];
	int result = 0;
	const gps_abstime start = gps_absolute_time();
	const gps_abstime timeout_us = static_cast<gps_abstime>(timeout) * 1000ULL;
	gps_abstime effective_timeout_us = timeout_us;

	if ((_config_state != ConfigState::Complete)
	    && (effective_timeout_us < (ConfigRetryIntervalUs + 500000ULL))) {
		effective_timeout_us = ConfigRetryIntervalUs + 500000ULL;
	}

	updateConfiguration();

	while (gps_absolute_time() - start < effective_timeout_us) {
		const int bytes_read = read(buffer, sizeof(buffer), timeout);

		if (bytes_read < 0) {
			return bytes_read;
		}

		for (int i = 0; i < bytes_read; ++i) {
			if (parseByte(buffer[i])) {
				result |= 1;
			}
		}

		updateConfiguration();

		if (result != 0) {
			return result;
		}
	}

	return -1;
}

void
GPSDriverJavadGreis::resetParser()
{
	_parse_state = ParseState::Id1;
	_message = Message::Unknown;
	_id[0] = 0;
	_id[1] = 0;
	_length_ascii[0] = 0;
	_length_ascii[1] = 0;
	_length_ascii[2] = 0;
	_payload_length = 0;
	_payload_bytes_read = 0;
	_payload_bytes_stored = 0;
	_checksum = 0;
	_checksum_bytes[0] = 0;
	_checksum_bytes[1] = 0;
	_checksum_length = 0;
	_minimum_payload_length = 0;
}

void
GPSDriverJavadGreis::restartParser(uint8_t byte)
{
	resetParser();

	if ((byte == '\r') || (byte == '\n')) {
		return;
	}

	if (isMessageIdByte(byte)) {
		_id[0] = static_cast<char>(byte);
		_parse_state = ParseState::Id2;
	}
}

bool
GPSDriverJavadGreis::parseByte(uint8_t byte)
{
	switch (_parse_state) {
	case ParseState::Id1:
		if ((byte != '\r') && (byte != '\n') && isMessageIdByte(byte)) {
			_id[0] = static_cast<char>(byte);
			_parse_state = ParseState::Id2;
		}

		return false;

	case ParseState::Id2:
		if (isMessageIdByte(byte)) {
			_id[1] = static_cast<char>(byte);
			_parse_state = ParseState::Length1;

		} else {
			restartParser(byte);
		}

		return false;

	case ParseState::Length1: {
			const int8_t value = asciiHexValue(byte);

			if (value < 0) {
				restartParser(byte);
				return false;
			}

			_length_ascii[0] = static_cast<char>(byte);
			_payload_length = static_cast<uint16_t>(value) << 8U;
			_parse_state = ParseState::Length2;
			return false;
		}

	case ParseState::Length2: {
			const int8_t value = asciiHexValue(byte);

			if (value < 0) {
				restartParser(byte);
				return false;
			}

			_length_ascii[1] = static_cast<char>(byte);
			_payload_length |= static_cast<uint16_t>(value) << 4U;
			_parse_state = ParseState::Length3;
			return false;
		}

	case ParseState::Length3: {
			const int8_t value = asciiHexValue(byte);

			if (value < 0) {
				restartParser(byte);
				return false;
			}

			_length_ascii[2] = static_cast<char>(byte);
			_payload_length |= static_cast<uint16_t>(value);

			if (_payload_length > MaxPayloadLength) {
				restartParser(byte);
				return false;
			}

			const MessageInfo info = messageInfo(_id);
			_message = info.message;
			_minimum_payload_length = info.minimum_length;
			_checksum_length = info.checksum_length;
			_payload_bytes_read = 0;
			_payload_bytes_stored = 0;
			_checksum_bytes[0] = 0;
			_checksum_bytes[1] = 0;
			startChecksum();

			if (_payload_length == 0) {
				const bool publish = (_message != Message::Unknown) ? dispatchMessage() : false;
				resetParser();
				return publish;
			}

			_parse_state = ParseState::Body;
			return false;
		}

	case ParseState::Body:
		if (_message != Message::Unknown) {
			if (_payload_bytes_stored < PayloadBufferLength) {
				_payload[_payload_bytes_stored++] = byte;
			}

			if (_checksum_length > 0) {
				const uint16_t covered_length = payloadDataLength();

				if (_payload_bytes_read < covered_length) {
					addChecksumByte(byte);

				} else {
					const uint16_t checksum_offset = _payload_bytes_read - covered_length;

					if (checksum_offset < sizeof(_checksum_bytes)) {
						_checksum_bytes[checksum_offset] = byte;
					}
				}
			}
		}

		_payload_bytes_read++;

		if (_payload_bytes_read >= _payload_length) {
			const bool publish = (_message != Message::Unknown) ? dispatchMessage() : false;
			resetParser();
			return publish;
		}

		return false;
	}

	resetParser();
	return false;
}

bool
GPSDriverJavadGreis::isMessageIdByte(uint8_t byte)
{
	return (byte >= 0x21U) && (byte <= 0x7eU);
}

int8_t
GPSDriverJavadGreis::asciiHexValue(uint8_t byte)
{
	if ((byte >= '0') && (byte <= '9')) {
		return static_cast<int8_t>(byte - '0');
	}

	if ((byte >= 'A') && (byte <= 'F')) {
		return static_cast<int8_t>(byte - 'A' + 10);
	}

	return -1;
}

uint8_t
GPSDriverJavadGreis::checksumRotate(uint8_t value)
{
	return static_cast<uint8_t>((value << 2U) | (value >> 6U));
}

void
GPSDriverJavadGreis::startChecksum()
{
	_checksum = 0;

	if ((_message == Message::Unknown) || (_checksum_length == 0)) {
		return;
	}

	addChecksumByte(static_cast<uint8_t>(_id[0]));
	addChecksumByte(static_cast<uint8_t>(_id[1]));
	addChecksumByte(static_cast<uint8_t>(_length_ascii[0]));
	addChecksumByte(static_cast<uint8_t>(_length_ascii[1]));
	addChecksumByte(static_cast<uint8_t>(_length_ascii[2]));
}

void
GPSDriverJavadGreis::addChecksumByte(uint8_t byte)
{
	_checksum = checksumRotate(_checksum) ^ byte;
}

bool
GPSDriverJavadGreis::checksumValid() const
{
	if (_checksum_length == 0) {
		return true;
	}

	if (_payload_length < _checksum_length) {
		return false;
	}

	const uint8_t checksum = checksumRotate(_checksum);

	if (_checksum_length == 1) {
		return checksum == _checksum_bytes[0];
	}

	if (_checksum_length == 2) {
		const int8_t high = asciiHexValue(_checksum_bytes[0]);
		const int8_t low = asciiHexValue(_checksum_bytes[1]);

		if ((high < 0) || (low < 0)) {
			return false;
		}

		return checksum == static_cast<uint8_t>((high << 4U) | low);
	}

	return false;
}

GPSDriverJavadGreis::MessageInfo
GPSDriverJavadGreis::messageInfo(const char id[2]) const
{
	if ((id[0] == 'G') && (id[1] == 'T')) {
		return {Message::GT, GTMinPayloadLength, 1};
	}

	if ((id[0] == 'S') && (id[1] == 'T')) {
		return {Message::ST, STMinPayloadLength, 1};
	}

	if (((id[0] == '~') && (id[1] == '~')) || ((id[0] == 'R') && (id[1] == 'T'))) {
		return {Message::RT, RTMinPayloadLength, 1};
	}

	if (((id[0] == ':') && (id[1] == ':')) || ((id[0] == 'E') && (id[1] == 'T'))) {
		return {Message::ET, ETMinPayloadLength, 1};
	}

	if ((id[0] == 'P') && (id[1] == 'G')) {
		return {Message::PG, PGMinPayloadLength, 1};
	}

	if ((id[0] == 'V') && (id[1] == 'G')) {
		return {Message::VG, VGMinPayloadLength, 1};
	}

	if ((id[0] == 'P') && (id[1] == 'V')) {
		return {Message::PV, PVMinPayloadLength, 1};
	}

	if ((id[0] == 'S') && (id[1] == 'G')) {
		return {Message::SG, SGMinPayloadLength, 1};
	}

	if ((id[0] == 'D') && (id[1] == 'P')) {
		return {Message::DP, DPMinPayloadLength, 1};
	}

	if ((id[0] == 'P') && (id[1] == 'S')) {
		return {Message::PS, PSMinPayloadLength, 1};
	}

	return {Message::Unknown, 0, 0};
}

uint16_t
GPSDriverJavadGreis::payloadDataLength() const
{
	return (_payload_length >= _checksum_length) ? static_cast<uint16_t>(_payload_length - _checksum_length) : 0;
}

bool
GPSDriverJavadGreis::dispatchMessage()
{
	if (_payload_length < _minimum_payload_length) {
		return false;
	}

	const uint16_t stored_length_limit = (_payload_length < PayloadBufferLength) ? _payload_length : PayloadBufferLength;

	if ((_payload_bytes_stored < stored_length_limit) && (_minimum_payload_length > _payload_bytes_stored)) {
		return false;
	}

	if (!checksumValid()) {
		return false;
	}

	bool decoded = false;

	switch (_message) {
	case Message::GT:
		decoded = decodeGT();
		break;

	case Message::RT:
		decoded = decodeRT();
		break;

	case Message::ST:
		decoded = decodeST();
		break;

	case Message::ET:
		decoded = decodeET();
		break;

	case Message::PG:
		decoded = decodePG();
		break;

	case Message::VG:
		decoded = decodeVG();
		break;

	case Message::PV:
		decoded = decodePV();
		break;

	case Message::SG:
		decoded = decodeSG();
		break;

	case Message::DP:
		decoded = decodeDP();
		break;

	case Message::PS:
		decoded = decodePS();
		break;

	case Message::Unknown:
		break;
	}

	return decoded && publishEpoch();
}

bool
GPSDriverJavadGreis::decodeGT()
{
	const uint32_t tow_ms = readU4(0);
	const uint32_t week = static_cast<uint32_t>(readU2(4)) + static_cast<uint32_t>(_payload[6]) * 1024U;

	_pending_gt.valid = true;
	_pending_gt.week = static_cast<uint16_t>(week);
	_pending_gt.tow_ms = tow_ms;

	if (_epoch.valid && _epoch.have_rt && !_epoch.have_gt) {
		_epoch.week = _pending_gt.week;
		_epoch.tow_ms = _pending_gt.tow_ms;
		_epoch.have_gt = true;
		_pending_gt.valid = false;
	}

	markSeen(SeenGT);
	return true;
}

bool
GPSDriverJavadGreis::decodeRT()
{
	const uint32_t time_ms = readU4(0);

	resetEpoch();
	_epoch.valid = true;
	_epoch.have_rt = true;
	_epoch.rt_time_ms = time_ms;

	if (_pending_gt.valid) {
		_epoch.week = _pending_gt.week;
		_epoch.tow_ms = _pending_gt.tow_ms;
		_epoch.have_gt = true;
		_pending_gt.valid = false;
	}

	markSeen(SeenRT);
	return true;
}

bool
GPSDriverJavadGreis::decodeST()
{
	const uint32_t time_ms = readU4(0);
	const uint8_t solution_type = _payload[4];

	markSeen(SeenST);

	if (!_epoch.valid || !_epoch.have_rt || _epoch.have_et || _epoch.published) {
		return true;
	}

	_epoch.st_time_ms = time_ms;

	if (time_ms != _epoch.rt_time_ms) {
		return true;
	}

	_epoch.have_st = true;
	_epoch.st_sol_type = solution_type;
	return true;
}

bool
GPSDriverJavadGreis::decodeET()
{
	const uint32_t time_ms = readU4(0);

	markSeen(SeenET);

	if (!_epoch.valid || !_epoch.have_rt || _epoch.published) {
		return true;
	}

	_epoch.et_time_ms = time_ms;
	_epoch.have_et = true;
	return true;
}

bool
GPSDriverJavadGreis::decodePG()
{
	const double latitude_rad = readDouble(0);
	const double longitude_rad = readDouble(8);
	const double height_m = readDouble(16);
	const float position_sigma_m = readFloat(24);
	const uint8_t solution_type = _payload[28];

	if (!std::isfinite(latitude_rad) || !std::isfinite(longitude_rad) || !std::isfinite(height_m)
	    || (latitude_rad < -90.0 * kDegToRad) || (latitude_rad > 90.0 * kDegToRad)
	    || (longitude_rad < -180.0 * kDegToRad) || (longitude_rad > 180.0 * kDegToRad)) {
		return false;
	}

	markSeen(SeenPG);

	if (!epochAcceptsData()) {
		return true;
	}

	_epoch.latitude_deg = latitude_rad * kRadToDeg;
	_epoch.longitude_deg = longitude_rad * kRadToDeg;
	_epoch.ellipsoid_height_m = height_m;
	_epoch.position_sigma_m = position_sigma_m;
	_epoch.position_sol_type = solution_type;
	_epoch.have_position = true;
	return true;
}

bool
GPSDriverJavadGreis::decodeVG()
{
	const float north_m_s = readFloat(0);
	const float east_m_s = readFloat(4);
	const float up_m_s = readFloat(8);
	const float velocity_sigma_m_s = readFloat(12);
	const uint8_t solution_type = _payload[16];

	if (!std::isfinite(north_m_s) || !std::isfinite(east_m_s) || !std::isfinite(up_m_s)) {
		return false;
	}

	markSeen(SeenVG);

	if (!epochAcceptsData()) {
		return true;
	}

	_epoch.velocity_ned.x = north_m_s;
	_epoch.velocity_ned.y = east_m_s;
	_epoch.velocity_ned.z = -up_m_s;
	_epoch.velocity_sigma_m_s = velocity_sigma_m_s;
	_epoch.velocity_sol_type = solution_type;
	_epoch.have_velocity = true;
	return true;
}

bool
GPSDriverJavadGreis::decodePV()
{
	const Vector3d position_ecef{readDouble(0), readDouble(8), readDouble(16)};
	const float position_sigma_m = readFloat(24);
	const Vector3f velocity_ecef{readFloat(28), readFloat(32), readFloat(36)};
	const float velocity_sigma_m_s = readFloat(40);
	const uint8_t solution_type = _payload[44];

	if (!std::isfinite(position_ecef.x) || !std::isfinite(position_ecef.y) || !std::isfinite(position_ecef.z)
	    || !std::isfinite(velocity_ecef.x) || !std::isfinite(velocity_ecef.y) || !std::isfinite(velocity_ecef.z)) {
		return false;
	}

	markSeen(SeenPV);

	if (!epochAcceptsData()) {
		return true;
	}

	_epoch.pv_position_ecef = position_ecef;
	_epoch.pv_velocity_ecef = velocity_ecef;
	_epoch.pv_position_sigma_m = position_sigma_m;
	_epoch.pv_velocity_sigma_m_s = velocity_sigma_m_s;
	_epoch.pv_sol_type = solution_type;
	_epoch.have_pv = true;
	_epoch.have_pv_geodetic = false;

	const double radius = std::sqrt(position_ecef.x * position_ecef.x
					+ position_ecef.y * position_ecef.y
					+ position_ecef.z * position_ecef.z);

	if (!std::isfinite(radius) || (radius < kWgs84A * 0.5) || (radius > kWgs84A * 2.0)) {
		return true;
	}

	double latitude_deg = 0.0;
	double longitude_deg = 0.0;
	float altitude_m = 0.0f;
	ECEF2lla(position_ecef.x, position_ecef.y, position_ecef.z, latitude_deg, longitude_deg, altitude_m);

	Vector3f velocity_ned{};
	const double latitude_rad = latitude_deg * kDegToRad;
	const double longitude_rad = longitude_deg * kDegToRad;

	if (!std::isfinite(latitude_deg) || !std::isfinite(longitude_deg) || !std::isfinite(altitude_m)
	    || (latitude_deg < -90.0) || (latitude_deg > 90.0)
	    || (longitude_deg < -180.0) || (longitude_deg > 180.0)
	    || !ecefVelocityToNed(velocity_ecef, latitude_rad, longitude_rad, velocity_ned)) {
		return true;
	}

	_epoch.pv_latitude_deg = latitude_deg;
	_epoch.pv_longitude_deg = longitude_deg;
	_epoch.pv_ellipsoid_height_m = static_cast<double>(altitude_m);
	_epoch.pv_velocity_ned = velocity_ned;
	_epoch.have_pv_geodetic = true;
	return true;
}

bool
GPSDriverJavadGreis::decodeSG()
{
	const float horizontal_accuracy_m = readFloat(0);
	const float vertical_accuracy_m = readFloat(4);
	const float speed_accuracy_m_s = readFloat(8);
	const uint8_t solution_type = _payload[16];

	markSeen(SeenSG);

	if (!epochAcceptsData()) {
		return true;
	}

	if (!std::isfinite(horizontal_accuracy_m) || !std::isfinite(vertical_accuracy_m) || !std::isfinite(speed_accuracy_m_s)
	    || (horizontal_accuracy_m < 0.0f) || (vertical_accuracy_m < 0.0f) || (speed_accuracy_m_s < 0.0f)) {
		return true;
	}

	_epoch.horizontal_accuracy_m = horizontal_accuracy_m;
	_epoch.vertical_accuracy_m = vertical_accuracy_m;
	_epoch.speed_accuracy_m_s = speed_accuracy_m_s;
	_epoch.sg_sol_type = solution_type;
	_epoch.have_sg = true;
	return true;
}

bool
GPSDriverJavadGreis::decodeDP()
{
	const float hdop = readFloat(0);
	const float vdop = readFloat(4);
	const uint8_t solution_type = _payload[12];

	markSeen(SeenDP);

	if (!epochAcceptsData()) {
		return true;
	}

	if (!std::isfinite(hdop) || !std::isfinite(vdop) || (hdop < 0.0f) || (vdop < 0.0f)) {
		return true;
	}

	_epoch.hdop = hdop;
	_epoch.vdop = vdop;
	_epoch.dp_sol_type = solution_type;
	_epoch.have_dp = true;
	return true;
}

bool
GPSDriverJavadGreis::decodePS()
{
	const uint8_t solution_type = _payload[0];
	uint16_t satellites_used = static_cast<uint16_t>(_payload[5]) + static_cast<uint16_t>(_payload[6]);
	const uint16_t data_length = payloadDataLength();
	const uint16_t  available_extra_systems = (data_length>8) ? static_cast<uint16_t>((data_length - 8) /3U) : 0U;
        const uint16_t extra_system_count =
        (available_extra_systems < PSKnownExtraSystems) ? available_extra_systems : PSKnownExtraSystems;
 

	for (uint16_t i = 0; i < extra_system_count; ++i) {
		const uint16_t used_offset = static_cast<uint16_t>(8U + i * 3U + 2U);

		if (used_offset < _payload_bytes_stored) {
			satellites_used += _payload[used_offset];
		}
	}

	markSeen(SeenPS);

	if (!epochAcceptsData()) {
		return true;
	}

	_epoch.satellites_used = 
        static_cast<uint8_t>((satellites_used < UINT8_MAX) ? satellites_used : UINT8_MAX); 
	_epoch.ps_sol_type = solution_type;
	_epoch.have_ps = true;
	return true;
}

void
GPSDriverJavadGreis::resetEpoch()
{
	_epoch = {};
	_epoch.position_sigma_m = -1.0f;
	_epoch.velocity_sigma_m_s = -1.0f;
	_epoch.pv_position_sigma_m = -1.0f;
	_epoch.pv_velocity_sigma_m_s = -1.0f;
	_epoch.hdop = kUnknownDop;
	_epoch.vdop = kUnknownDop;
}

bool
GPSDriverJavadGreis::epochAcceptsData() const
{
	return _epoch.valid && _epoch.have_rt && !_epoch.have_et && !_epoch.published;
}

void
GPSDriverJavadGreis::markSeen(uint32_t bit)
{
	_seen_messages |= bit;

	if (requiredMessagesSeen()) {
		_config_state = ConfigState::Complete;
	}
}

bool
GPSDriverJavadGreis::requiredMessagesSeen() const
{
	constexpr uint32_t required = SeenGT | SeenRT | SeenST | SeenET | SeenPG | SeenVG;
	return (_seen_messages & required) == required;
}

bool
GPSDriverJavadGreis::epochComplete() const
{
	const bool have_primary_navigation = _epoch.have_position && _epoch.have_velocity;
	const bool have_fallback_navigation = _epoch.have_pv_geodetic;

	return _epoch.valid
	       && _epoch.have_rt
	       && _epoch.have_st
	       && _epoch.have_et
	       && _epoch.have_gt
	       && (have_primary_navigation || have_fallback_navigation)
	       && !_epoch.published;
}

bool
GPSDriverJavadGreis::publishEpoch()
{
	if ((_gps_position == nullptr) || !epochComplete()) {
		return false;
	}

	const bool use_primary_navigation = _epoch.have_position && _epoch.have_velocity;
	const bool use_pv_navigation = !use_primary_navigation && _epoch.have_pv_geodetic;

	if (!use_primary_navigation && !use_pv_navigation) {
		return false;
	}

	uint8_t fix_type = sensor_gps_s::FIX_TYPE_NONE;
	bool have_fix_type = false;
	const auto considerSolution = [&fix_type, &have_fix_type](uint8_t solution_type) {
		const uint8_t candidate = solutionTypeToFix(solution_type);

		if (!have_fix_type || (candidate < fix_type)) {
			fix_type = candidate;
			have_fix_type = true;
		}
	};

	considerSolution(_epoch.st_sol_type);

	if (use_primary_navigation) {
		considerSolution(_epoch.position_sol_type);
		considerSolution(_epoch.velocity_sol_type);

	} else {
		considerSolution(_epoch.pv_sol_type);
	}

	if (_epoch.have_sg) {
		considerSolution(_epoch.sg_sol_type);
	}

	if (_epoch.have_dp) {
		considerSolution(_epoch.dp_sol_type);
	}

	if (_epoch.have_ps) {
		considerSolution(_epoch.ps_sol_type);
	}

	const double latitude_deg = use_pv_navigation ? _epoch.pv_latitude_deg : _epoch.latitude_deg;
	const double longitude_deg = use_pv_navigation ? _epoch.pv_longitude_deg : _epoch.longitude_deg;
	const double ellipsoid_height_m = use_pv_navigation ? _epoch.pv_ellipsoid_height_m : _epoch.ellipsoid_height_m;
	const Vector3f velocity_ned = use_pv_navigation ? _epoch.pv_velocity_ned : _epoch.velocity_ned;
	const float position_sigma_m = use_pv_navigation ? _epoch.pv_position_sigma_m : _epoch.position_sigma_m;
	const float velocity_sigma_m_s = use_pv_navigation ? _epoch.pv_velocity_sigma_m_s : _epoch.velocity_sigma_m_s;
	const float horizontal_speed_m_s = std::sqrt(velocity_ned.x * velocity_ned.x + velocity_ned.y * velocity_ned.y);

	_gps_position->timestamp = gps_absolute_time();
	_gps_position->timestamp_sample = _gps_position->timestamp;
	_gps_position->time_utc_usec = 0;
	_gps_position->timestamp_time_relative = 0;

	_gps_position->latitude_deg = latitude_deg;
	_gps_position->longitude_deg = longitude_deg;
	_gps_position->altitude_ellipsoid_m = ellipsoid_height_m;
	// The configured GREIS stream provides ellipsoid height but not geoid separation.
	// PX4 has no "MSL unavailable" flag, so keep the height finite and document this
	// receiver-specific limitation instead of adding an unverified geoid correction.
	_gps_position->altitude_msl_m = ellipsoid_height_m;

	_gps_position->fix_type = fix_type;
	_gps_position->vel_n_m_s = velocity_ned.x;
	_gps_position->vel_e_m_s = velocity_ned.y;
	_gps_position->vel_d_m_s = velocity_ned.z;
	_gps_position->vel_m_s = horizontal_speed_m_s;
	_gps_position->cog_rad = std::atan2(velocity_ned.y, velocity_ned.x);
	_gps_position->c_variance_rad = kFallbackCourseVarianceRad;
	_gps_position->vel_ned_valid = fix_type > sensor_gps_s::FIX_TYPE_NONE;

	_gps_position->satellites_used = _epoch.have_ps ? _epoch.satellites_used : 0;
	_gps_position->hdop = _epoch.have_dp ? _epoch.hdop : kUnknownDop;
	_gps_position->vdop = _epoch.have_dp ? _epoch.vdop : kUnknownDop;

	if (_epoch.have_sg) {
		_gps_position->eph = _epoch.horizontal_accuracy_m;
		_gps_position->epv = _epoch.vertical_accuracy_m;
		_gps_position->s_variance_m_s = _epoch.speed_accuracy_m_s;

	} else {
		if (std::isfinite(position_sigma_m) && (position_sigma_m >= 0.0f)) {
			_gps_position->eph = position_sigma_m;
			_gps_position->epv = position_sigma_m;

		} else {
			_gps_position->eph = 0.0f;
			_gps_position->epv = 0.0f;
		}

		if (std::isfinite(velocity_sigma_m_s) && (velocity_sigma_m_s >= 0.0f)) {
			_gps_position->s_variance_m_s = velocity_sigma_m_s;

		} else {
			_gps_position->s_variance_m_s = 0.0f;
		}
	}

	_rate_count_lat_lon++;
	_rate_count_vel++;
	_epoch.published = true;
	return true;
}

void
GPSDriverJavadGreis::updateConfiguration(bool force)
{
	if (_config_state == ConfigState::Complete) {
		return;
	}

	if (requiredMessagesSeen()) {
		_config_state = ConfigState::Complete;
		return;
	}

	const gps_abstime now = gps_absolute_time();

	if (!force && (now < _next_config_time_us)) {
		return;
	}

	switch (_config_state) {
	case ConfigState::DisableOutput:
		if (sendCommand(kDisableOutputCommand)) {
			_config_state = ConfigState::EnableStream;
			_next_config_time_us = now + InitialConfigDelayUs;
		}

		break;

	case ConfigState::EnableStream:
		if (sendCommand(kEnableStreamCommand)) {
			_next_config_time_us = now + ConfigRetryIntervalUs;
		}

		break;

	case ConfigState::Complete:
		break;
	}
}

bool
GPSDriverJavadGreis::sendCommand(const char *command)
{
	const int length = static_cast<int>(std::strlen(command));
	return write(command, length) == length;
}

uint16_t
GPSDriverJavadGreis::readU2(uint16_t offset) const
{
	return static_cast<uint16_t>(_payload[offset])
	       | (static_cast<uint16_t>(_payload[offset + 1U]) << 8U);
}

uint32_t
GPSDriverJavadGreis::readU4(uint16_t offset) const
{
	return static_cast<uint32_t>(_payload[offset])
	       | (static_cast<uint32_t>(_payload[offset + 1U]) << 8U)
	       | (static_cast<uint32_t>(_payload[offset + 2U]) << 16U)
	       | (static_cast<uint32_t>(_payload[offset + 3U]) << 24U);
}

float
GPSDriverJavadGreis::readFloat(uint16_t offset) const
{
	const uint32_t raw = readU4(offset);
	float value = 0.0f;
	std::memcpy(&value, &raw, sizeof(value));
	return value;
}

double
GPSDriverJavadGreis::readDouble(uint16_t offset) const
{
	uint64_t raw = 0;

	for (uint8_t i = 0; i < 8; ++i) {
		raw |= static_cast<uint64_t>(_payload[offset + i]) << (8U * i);
	}

	double value = 0.0;
	std::memcpy(&value, &raw, sizeof(value));
	return value;
}

uint8_t
GPSDriverJavadGreis::solutionTypeToFix(uint8_t solution_type)
{
	switch (solution_type) {
	case 1:
		return sensor_gps_s::FIX_TYPE_3D;

	case 2:
		return sensor_gps_s::FIX_TYPE_RTCM_CODE_DIFFERENTIAL;

	case 3:
		return sensor_gps_s::FIX_TYPE_RTK_FLOAT;

	case 4:
		return sensor_gps_s::FIX_TYPE_RTK_FIXED;

	case 5:
		// JAVAD reports a static solution here in the ArduPilot reference, but
		// sensor_gps has no static/base-station fix type. Keep it conservative.
		return sensor_gps_s::FIX_TYPE_3D;

	case 0:
	default:
		return sensor_gps_s::FIX_TYPE_NONE;
	}
}

bool
GPSDriverJavadGreis::ecefVelocityToNed(const Vector3f &ecef_velocity, double latitude_rad, double longitude_rad,
				       Vector3f &ned_velocity)
{
	if (!std::isfinite(ecef_velocity.x) || !std::isfinite(ecef_velocity.y) || !std::isfinite(ecef_velocity.z)
	    || !std::isfinite(latitude_rad) || !std::isfinite(longitude_rad)) {
		return false;
	}

	const double sin_lat = std::sin(latitude_rad);
	const double cos_lat = std::cos(latitude_rad);
	const double sin_lon = std::sin(longitude_rad);
	const double cos_lon = std::cos(longitude_rad);

	const double vx = static_cast<double>(ecef_velocity.x);
        const double vy = static_cast<double>(ecef_velocity.y);
        const double vz = static_cast<double>(ecef_velocity.z);

        ned_velocity.x = static_cast<float>(
           -sin_lat * cos_lon * vx
           - sin_lat * sin_lon * vy
           + cos_lat * vz
        );

        ned_velocity.y = static_cast<float>(
         -sin_lon * vx
         + cos_lon * vy
        );

       ned_velocity.z = static_cast<float>(
        -cos_lat * cos_lon * vx
        - cos_lat * sin_lon * vy
        - sin_lat * vz
      );

	return std::isfinite(ned_velocity.x) && std::isfinite(ned_velocity.y) && std::isfinite(ned_velocity.z);
}
