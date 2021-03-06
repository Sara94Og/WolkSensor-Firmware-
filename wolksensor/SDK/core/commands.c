#include "commands.h"
#include "platform_specific.h"
#include "sensor_readings_buffer.h"
#include "system_buffer.h"
#include "mqtt_communication_protocol.h"
#include "wifi_communication_module.h"
#include "config.h"
#include "chrono.h"
#include "logger.h"
#include "system.h"
#include "sensors.h"
#include "protocol.h"
#include "global_dependencies.h"
#include "commands_dependencies.h"
#include "wolksensor_dependencies.h"

commands_dependencies_t commands_dependencies;

void init_commands(void)
{
	memset(&commands_dependencies, 0, sizeof(commands_dependencies_t));
}

command_execution_result_t cmd_now(command_t* command, circular_buffer_t* response_buffer)
{
	LOG(1, "Executing command NOW");
	
	append_done(response_buffer);
	
	if(commands_dependencies.exchange_data) commands_dependencies.exchange_data();
	
	return COMMAND_EXECUTED_SUCCESSFULLY;
}

command_execution_result_t cmd_reload(command_t* command, circular_buffer_t* response_buffer)
{
	LOG(1, "Executing command RELOAD");
	
	append_done(response_buffer);
	
	if(commands_dependencies.reset) commands_dependencies.reset();
	
	return COMMAND_EXECUTED_SUCCESSFULLY;
}

command_execution_result_t cmd_heartbeat(command_t* command, circular_buffer_t* response_buffer)
{
	LOG(1, "Executing command HEARTBEAT");
	
	if(command->has_argument && (system_heartbeat != command->argument.uint32_argument))
	{
		LOG_PRINT(1, PSTR("Setting heartbeat: %u\r\n"), command->argument.uint32_argument);
		system_heartbeat = command->argument.uint32_argument;
		global_dependencies.config_write(&system_heartbeat, CFG_SYSTEM_HEARTBEAT, 1, sizeof(system_heartbeat));
		
		if(commands_dependencies.start_heartbeat) commands_dependencies.start_heartbeat(system_heartbeat);
	}
	
	append_heartbeat(system_heartbeat, response_buffer);
	
	return COMMAND_EXECUTED_SUCCESSFULLY;
}

command_execution_result_t cmd_rtc(command_t* command, circular_buffer_t* response_buffer)
{
	LOG(1, "Executing command RTC");
	
	if(command->has_argument)
	{
		LOG(1, "Setting rtc");
		rtc_set(command->argument.uint32_argument);
	}
	//else
	//{
	//	int16_t readings[NUMBER_OF_SENSORS];
	//	readings[1] = 20;
	//	store_sensor_readings(readings);
	//}
	
	append_rtc(rtc_get_ts(), response_buffer);
	
	return COMMAND_EXECUTED_SUCCESSFULLY;
}

command_execution_result_t cmd_status(command_t* command, circular_buffer_t* response_buffer)
{
	LOG(1, "Executing command STATUS");

	char status[128];
	memset(status, 0, 128);
	
	if(commands_dependencies.start_heartbeat) commands_dependencies.get_application_status(status, 128);
	
	append_status(status, response_buffer);

	return COMMAND_EXECUTED_SUCCESSFULLY;
}

command_execution_result_t cmd_version(command_t* command, circular_buffer_t* response_buffer)
{	
	LOG(1, "Executing command VERSION");
		
	char version[9];
	sprintf_P(version, PSTR("%d.%d.%d"), FW_VERSION_MAJOR, FW_VERSION_MINOR, FW_VERSION_PATCH);
	
	append_version(version, response_buffer);
	
	return COMMAND_EXECUTED_SUCCESSFULLY;
}

command_execution_result_t cmd_id(command_t* command, circular_buffer_t* response_buffer)
{
	LOG(1, "Executing command ID");
	
	if(command->has_argument)
	{
		if(!(*device_id))
		{
			strncpy((char *)device_id, command->argument.string_argument, sizeof(device_id));
			global_dependencies.config_write(device_id, CFG_DEVICE_ID, 1, sizeof(device_id));
		}
		else
		{
			append_bad_request(response_buffer);
			return COMMAND_EXECUTED_SUCCESSFULLY;
		}
	}
	
	append_id(device_id, response_buffer);
	return COMMAND_EXECUTED_SUCCESSFULLY;
}

command_execution_result_t cmd_signature(command_t* command, circular_buffer_t* response_buffer)
{
	LOG(1, "Executing command SIGNATURE");
	
	if(command->has_argument)
	{
		if(!(*device_preshared_key))
		{
			strncpy((char *)device_preshared_key, command->argument.string_argument, sizeof(device_preshared_key));
			global_dependencies.config_write(device_preshared_key, CFG_DEVICE_PRESHARED_KEY, 1, sizeof(device_preshared_key));
		}
		else
		{
			append_bad_request(response_buffer);
			return COMMAND_EXECUTED_SUCCESSFULLY;
		}
	}
	
	append_signature(device_preshared_key, response_buffer);
	return COMMAND_EXECUTED_SUCCESSFULLY;
}

command_execution_result_t cmd_movement(command_t* command, circular_buffer_t* response_buffer)
{
	LOG(1, "Executing command MOVEMENT");
	
	if(command->has_argument && (movement_status != command->argument.bool_argument))
	{
		movement_status = command->argument.bool_argument;
		global_dependencies.config_write(&movement_status, CFG_MOVEMENT, 1, sizeof(movement_status));
		
		if(movement_status)
		{
			wolksensor_dependencies.enable_movement();
		}
		else
		{
			wolksensor_dependencies.disable_movement();
		}
	}
	
	append_movement_enabled(movement_status, response_buffer);
	return COMMAND_EXECUTED_SUCCESSFULLY;
}

command_execution_result_t cmd_atmo(command_t* command, circular_buffer_t* response_buffer)
{
	LOG(1, "Executing command ATMO");
	
	if(command->has_argument && (atmo_status != command->argument.bool_argument))
	{
		atmo_status = command->argument.bool_argument;
		global_dependencies.config_write(&atmo_status, CFG_ATMO, 1, sizeof(atmo_status));
	}
	
	append_atmo_enabled(atmo_status, response_buffer);
	return COMMAND_EXECUTED_SUCCESSFULLY;
}

command_execution_result_t cmd_system(command_t* command, circular_buffer_t* response_buffer)
{
	LOG(1, "Executing command SYSTEM");
	
	if(command->has_argument && command->argument.bool_argument)
	{
		system_buffer_clear();
		command->argument.uint16_argument = 0;
	}
	
	command->argument.uint16_argument += append_system_info(&system_buffer, command->argument.uint16_argument, response_buffer, true);
	return (command->argument.uint16_argument == circular_buffer_size(&system_buffer)) ? COMMAND_EXECUTED_SUCCESSFULLY : COMMAND_EXECUTED_PARTIALLY;
}

command_execution_result_t cmd_readings(command_t* command, circular_buffer_t* response_buffer)
{
	LOG(1, "Executing command READINGS");
	
	if(command->has_argument && command->argument.bool_argument)
	{
		sensor_readings_buffer_clear();
		command->argument.uint16_argument = 0;
	}
	
	command->argument.uint16_argument += append_sensor_readings(&sensor_readings_buffer, command->argument.uint16_argument, response_buffer, true);
	return (command->argument.uint16_argument == circular_buffer_size(&sensor_readings_buffer)) ? COMMAND_EXECUTED_SUCCESSFULLY : COMMAND_EXECUTED_PARTIALLY;
}

command_execution_result_t cmd_alarm(command_t* command, circular_buffer_t* response_buffer)
{
	LOG(1, "Executing command ALARM");
	
	if(command->has_argument)
	{
		memcpy(&sensors_alarms, &command->argument.sensors_alarms_argument, sizeof(sensors_alarms));
	}
	
	append_alarms(sensors_alarms, NUMBER_OF_SENSORS, response_buffer);
	
	return COMMAND_EXECUTED_SUCCESSFULLY;
}

command_execution_result_t cmd_mac(command_t* command, circular_buffer_t* response_buffer)
{
	LOG(1, "Executing command MAC");
	append_mac_address(mac_address_nwmem, response_buffer);
	return COMMAND_EXECUTED_SUCCESSFULLY;
}

command_execution_result_t cmd_url(command_t* command, circular_buffer_t* response_buffer)
{
	LOG(1, "Executing command URL");
	
	if (command->has_argument)
	{
		strncpy(server_ip, command->argument.string_argument, sizeof(server_ip));
		global_dependencies.config_write(server_ip, CFG_SERVER_IP, 1, sizeof(server_ip));
		
		if(commands_dependencies.communication_module_close_socket)
		{
			communication_module_process_handle_t handle = commands_dependencies.communication_module_close_socket();
			while(handle());
		}
	}
	
	append_url(server_ip, response_buffer);
	return COMMAND_EXECUTED_SUCCESSFULLY;
}

command_execution_result_t cmd_port(command_t* command, circular_buffer_t* response_buffer)
{
	LOG(1, "Executing command PORT");
	
	if (command->has_argument)
	{
		server_port = command->argument.uint32_argument;
		global_dependencies.config_write(&server_port, CFG_SERVER_PORT, 1, sizeof(server_port));
		
		if(commands_dependencies.communication_module_close_socket)
		{
			communication_module_process_handle_t handle = commands_dependencies.communication_module_close_socket();
			while(handle());
		}
	}
	
	append_port(server_port, response_buffer);
	return COMMAND_EXECUTED_SUCCESSFULLY;
}

command_execution_result_t cmd_ssid(command_t* command, circular_buffer_t* response_buffer)
{
	LOG(1, "Executing command SSID");
	
	if (command->has_argument)
	{
		if (strcmp_P(command->argument.string_argument, PSTR("NULL")) == 0)
		{
			memset(wifi_ssid, 0, sizeof(wifi_ssid));
		}
		else
		{
			strncpy(wifi_ssid, command->argument.string_argument, sizeof(wifi_ssid));
		}
		global_dependencies.config_write(wifi_ssid, CFG_WIFI_SSID, 1, sizeof(wifi_ssid));
		
		if(commands_dependencies.wifi_communication_module_disconnect)
		{
			communication_module_process_handle_t handle = commands_dependencies.wifi_communication_module_disconnect();
			while(handle());
		}
	}
	
	append_ssid(wifi_ssid, response_buffer);
	return COMMAND_EXECUTED_SUCCESSFULLY;
}

command_execution_result_t cmd_pass(command_t* command, circular_buffer_t* response_buffer)
{
	LOG(1, "Executing command PASS");
	
	if (command->has_argument)
	{
		if (strcmp_P(command->argument.string_argument, PSTR("NULL")) == 0)
		{
			memset(wifi_password, 0, sizeof(wifi_password));
		}
		else
		{
			strncpy(wifi_password, command->argument.string_argument, sizeof(wifi_password));
		}
		global_dependencies.config_write(&wifi_password, CFG_WIFI_PASS, 1, sizeof(wifi_password));
		
		if(commands_dependencies.wifi_communication_module_disconnect)
		{
			communication_module_process_handle_t handle = commands_dependencies.wifi_communication_module_disconnect();
			while(handle());
		}
	}
	
	append_pass(wifi_password, response_buffer);
	return COMMAND_EXECUTED_SUCCESSFULLY;
}

command_execution_result_t cmd_auth(command_t* command, circular_buffer_t* response_buffer)
{
	LOG(1, "Executing command AUTH");
	
	if (command->has_argument) {
		wifi_auth_type = command->argument.uint32_argument;
		global_dependencies.config_write(&wifi_auth_type, CFG_WIFI_AUTH, 1, sizeof(wifi_auth_type));
		
		if(commands_dependencies.wifi_communication_module_disconnect)
		{
			communication_module_process_handle_t handle = commands_dependencies.wifi_communication_module_disconnect();
			while(handle());
		}
	}
	
	append_auth(wifi_auth_type, response_buffer);
	return COMMAND_EXECUTED_SUCCESSFULLY;
}

command_execution_result_t cmd_static_ip(command_t* command, circular_buffer_t* response_buffer)
{
	LOG(1, "Executing command STATIC_IP");
	
	if(command->has_argument)
	{
		if(strcmp_P(command->argument.string_argument, PSTR("OFF")) == 0)
		{
			memset(wifi_static_ip, 0, sizeof(wifi_static_ip));
			global_dependencies.config_write(&wifi_static_ip, CFG_WIFI_STATIC_IP, 1, sizeof(wifi_static_ip));
			
			memset(wifi_static_mask, 0, sizeof(wifi_static_mask));
			global_dependencies.config_write(&wifi_static_mask, CFG_WIFI_STATIC_MASK, 1, sizeof(wifi_static_mask));
			
			memset(wifi_static_gateway, 0, sizeof(wifi_static_gateway));
			global_dependencies.config_write(&wifi_static_gateway, CFG_WIFI_STATIC_GATEWAY, 1, sizeof(wifi_static_gateway));
			
			memset(wifi_static_dns, 0, sizeof(wifi_static_dns));
			global_dependencies.config_write(&wifi_static_dns, CFG_WIFI_STATIC_DNS, 1, sizeof(wifi_static_dns));
			
			LOG(1, "Dynamic IP set");
			
			communication_module_process_handle_t handle = commands_dependencies.wifi_communication_module_disconnect();
			while(handle());
		}
		else
		{
			strcpy(wifi_static_ip, command->argument.string_argument);
			global_dependencies.config_write(&wifi_static_ip, CFG_WIFI_STATIC_IP, 1, sizeof(wifi_static_ip));
			
			if(commands_dependencies.is_static_ip_set && commands_dependencies.wifi_communication_module_disconnect)
			{
				if(commands_dependencies.is_static_ip_set())
				{
					communication_module_process_handle_t handle = commands_dependencies.wifi_communication_module_disconnect();
					while(handle());
				}
			}
		}
	}
	
	append_static_ip(wifi_static_ip, response_buffer);
	return COMMAND_EXECUTED_SUCCESSFULLY;
}

command_execution_result_t cmd_static_mask(command_t* command, circular_buffer_t* response_buffer)
{
	LOG(1, "Executing command STATIC_MASK");
	
	if(command->has_argument)
	{
		strcpy(wifi_static_mask, command->argument.string_argument);
		global_dependencies.config_write(&wifi_static_mask, CFG_WIFI_STATIC_MASK, 1, sizeof(wifi_static_mask));
		
		if(commands_dependencies.is_static_ip_set && commands_dependencies.wifi_communication_module_disconnect)
		{
			if(commands_dependencies.is_static_ip_set())
			{
				communication_module_process_handle_t handle = commands_dependencies.wifi_communication_module_disconnect();
				while(handle());
			}
		}
	}
	
	append_static_mask(wifi_static_mask, response_buffer);
	return COMMAND_EXECUTED_SUCCESSFULLY;
}

command_execution_result_t cmd_static_gateway(command_t* command, circular_buffer_t* response_buffer)
{
	LOG(1, "Executing command STATIC_GATEWAY");
	
	if(command->has_argument)
	{
		strcpy(wifi_static_gateway, command->argument.string_argument);
		global_dependencies.config_write(&wifi_static_gateway, CFG_WIFI_STATIC_GATEWAY, 1, sizeof(wifi_static_gateway));
		
		if(commands_dependencies.is_static_ip_set && commands_dependencies.wifi_communication_module_disconnect)
		{
			if(commands_dependencies.is_static_ip_set())
			{
				communication_module_process_handle_t handle = commands_dependencies.wifi_communication_module_disconnect();
				while(handle());
			}
		}
	}
	
	append_static_gateway(wifi_static_gateway, response_buffer);
	return COMMAND_EXECUTED_SUCCESSFULLY;
}

command_execution_result_t cmd_static_dns(command_t* command, circular_buffer_t* response_buffer)
{
	LOG(1, "Executing command STATIC_DNS");
	
	if(command->has_argument)
	{
		strcpy(wifi_static_dns, command->argument.string_argument);
		global_dependencies.config_write(&wifi_static_dns, CFG_WIFI_STATIC_DNS, 1, sizeof(wifi_static_dns));
		
		if(commands_dependencies.is_static_ip_set && commands_dependencies.wifi_communication_module_disconnect)
		{
			if(commands_dependencies.is_static_ip_set())
			{
				communication_module_process_handle_t handle = commands_dependencies.wifi_communication_module_disconnect();
				while(handle());
			}
		}
	}
	
	append_static_dns(wifi_static_dns, response_buffer);
	
	return COMMAND_EXECUTED_SUCCESSFULLY;
}

command_execution_result_t cmd_set(command_t* command, circular_buffer_t* response_buffer)
{
	if(command->has_argument)
	{
		if(commands_dependencies.set_actuator && commands_dependencies.get_actuator_state)
		{
			commands_dependencies.set_actuator(command->argument.set_argument.id, command->argument.set_argument.target_value);
			commands_dependencies.get_actuator_state(command->argument.set_argument.id);
		}
	}
	else
	{
		if(commands_dependencies.get_actuator_state)
		{
			uint8_t i;
			for(i = 0; i < NUMBER_OF_ACTUATORS; i++)
			{
				commands_dependencies.get_actuator_state(actuators[i].id);
			}
		}
	}
	
	append_done(response_buffer);
	
	return COMMAND_EXECUTED_SUCCESSFULLY;
}

command_execution_result_t cmd_knx_physical_address(command_t* command, circular_buffer_t* response_buffer)
{
	LOG(1, "Executing command KNX_PHYSICAL_ADDRESS");
	
	if(command->has_argument)
	{
		memcpy(knx_physical_address, command->argument.knx_address_argument, 2);
		global_dependencies.config_write(&knx_physical_address, CFG_KNX_PHYSICAL_ADDRESS, 1, sizeof(knx_physical_address));
	}
	
	LOG_PRINT(1, PSTR("KNX physical address %02x%02x\r\n"), knx_physical_address[0], knx_physical_address[1]);
	append_knx_physical_address(knx_physical_address, response_buffer);
	
	return COMMAND_EXECUTED_SUCCESSFULLY;
}

command_execution_result_t cmd_knx_group_address(command_t* command, circular_buffer_t* response_buffer)
{
	LOG(1, "Executing command KNX_GROUP_ADDRESS");
	
	if(command->has_argument)
	{
		memcpy(knx_group_address, command->argument.knx_address_argument, 2);
		global_dependencies.config_write(&knx_group_address, CFG_KNX_GROUP_ADDRESS, 1, sizeof(knx_group_address));
	}
	
	LOG_PRINT(1, PSTR("KNX group address %02x%02x\r\n"), knx_group_address[0], knx_group_address[1]);
	append_knx_group_address(knx_group_address, response_buffer);
	
	return COMMAND_EXECUTED_SUCCESSFULLY;
}

command_execution_result_t cmd_multicast_address(command_t* command, circular_buffer_t* response_buffer)
{
	LOG(1, "Executing command MULTICAST_ADDRESS");
	
	if (command->has_argument)
	{
		strncpy(knx_multicast_address, command->argument.string_argument, sizeof(knx_multicast_address));
		global_dependencies.config_write(knx_multicast_address, CFG_KNX_MULTICAST_ADDRESS, 1, sizeof(knx_multicast_address));
		
		if(commands_dependencies.communication_module_close_socket)
		{
			communication_module_process_handle_t handle = commands_dependencies.communication_module_close_socket();
			while(handle());
		}
	}
	
	append_multicast_address(knx_multicast_address, response_buffer);
	return COMMAND_EXECUTED_SUCCESSFULLY;
}

command_execution_result_t cmd_multicast_port(command_t* command, circular_buffer_t* response_buffer)
{
	LOG(1, "Executing command MULTICAST_PORT");
	
	if (command->has_argument)
	{
		knx_multicast_port = command->argument.uint32_argument;
		global_dependencies.config_write(&knx_multicast_port, CFG_KNX_MULTICAST_PORT, 1, sizeof(knx_multicast_port));
		
		if(commands_dependencies.communication_module_close_socket)
		{
			communication_module_process_handle_t handle = commands_dependencies.communication_module_close_socket();
			while(handle());
		}
	}
	
	append_multicast_port(knx_multicast_port, response_buffer);
	return COMMAND_EXECUTED_SUCCESSFULLY;
}

command_execution_result_t cmd_knx_nat(command_t* command, circular_buffer_t* response_buffer)
{
	LOG(1, "Executing command KNX_NAT");
	
	if(command->has_argument && (knx_nat != command->argument.bool_argument))
	{
		knx_nat = command->argument.bool_argument;
		global_dependencies.config_write(&knx_nat, CFG_KNX_NAT, 1, sizeof(knx_nat));
	}
	
	append_knx_nat_status(knx_nat, response_buffer);
	return COMMAND_EXECUTED_SUCCESSFULLY;
}

command_execution_result_t cmd_location(command_t* command, circular_buffer_t* response_buffer)
{
	LOG(1, "Executing command LOCATION");
	
	if(command->has_argument && (location != command->argument.bool_argument))
	{
		location = command->argument.bool_argument;
		global_dependencies.config_write(&location, CFG_LOCATION, 1, sizeof(location));
	}
	
	append_location_status(location, response_buffer);
	return COMMAND_EXECUTED_SUCCESSFULLY;
}

command_execution_result_t cmd_ssl(command_t* command, circular_buffer_t* response_buffer)
{
	LOG(1, "Executing command SSL");
	
	if(command->has_argument && (ssl != command->argument.bool_argument))
	{
		ssl = command->argument.bool_argument;
		global_dependencies.config_write(&ssl, CFG_SSL, 1, sizeof(ssl));
	}
	
	append_ssl_status(ssl, response_buffer);
	return COMMAND_EXECUTED_SUCCESSFULLY;
}

command_execution_result_t execute_command(command_t* command, circular_buffer_t* response_buffer)
{
	switch(command->type)
	{
		case COMMAND_MAC:
		{
			return cmd_mac(command, response_buffer);
		}
		case COMMAND_NOW:
		{
			return cmd_now(command, response_buffer);
		}
		case COMMAND_RELOAD:
		{
			return cmd_reload(command, response_buffer);
		}
		case COMMAND_HEARTBEAT:
		{
			return cmd_heartbeat(command, response_buffer);
		}
		case COMMAND_RTC:
		{
			return cmd_rtc(command, response_buffer);
		}
		case COMMAND_VERSION:
		{
			return cmd_version(command, response_buffer);
		}
		case COMMAND_STATUS:
		{
			return cmd_status(command, response_buffer);
		}
		case COMMAND_READINGS:
		{
			return cmd_readings(command, response_buffer);
		}
		case COMMAND_ID:
		{
			return cmd_id(command, response_buffer);
		}
		case COMMAND_SIGNATURE:
		{
			return cmd_signature(command, response_buffer);
		}
		case COMMAND_URL:
		{
			return cmd_url(command, response_buffer);
		}
		case COMMAND_PORT:
		{
			return cmd_port(command, response_buffer);
		}
		case COMMAND_SSID:
		{
			return cmd_ssid(command, response_buffer);
		}
		case COMMAND_PASS:
		{
			return cmd_pass(command, response_buffer);
		}
		case COMMAND_AUTH:
		{
			return cmd_auth(command, response_buffer);
		}
		case COMMAND_MOVEMENT:
		{
			return cmd_movement(command, response_buffer);
		}
		case COMMAND_ATMO:
		{
			return cmd_atmo(command, response_buffer);
		}
		case COMMAND_SYSTEM:
		{
			return cmd_system(command, response_buffer);
		}
		case COMMAND_STATIC_IP:
		{
			return cmd_static_ip(command, response_buffer);
		}
		case COMMAND_STATIC_MASK:
		{
			return cmd_static_mask(command, response_buffer);
		}
		case COMMAND_STATIC_GATEWAY:
		{
			return cmd_static_gateway(command, response_buffer);
		}
		case COMMAND_STATIC_DNS:
		{
			return cmd_static_dns(command, response_buffer);
		}
		case COMMAND_ALARM:
		{
			return cmd_alarm(command, response_buffer);
		}
		case COMMAND_SET:
		{
			return cmd_set(command, response_buffer);
		}
		case COMMAND_KNX_PHYSICAL_ADDRESS:
		{
			return cmd_knx_physical_address(command, response_buffer);
		}
		case COMMAND_KNX_GROUP_ADDRESS:
		{
			return cmd_knx_group_address(command, response_buffer);
		}
		case COMMAND_KNX_MULTICAST_ADDRESS:
		{
			return cmd_multicast_address(command, response_buffer);
		}
		case COMMAND_KNX_MULTICAST_PORT:
		{
			return cmd_multicast_port(command, response_buffer);
		}
		case COMMAND_KNX_NAT:
		{
			return cmd_knx_nat(command, response_buffer);
		}
		case COMMAND_LOCATION:
		{
			return cmd_location(command, response_buffer);
		}
		case COMMAND_SSL:
		{
			return cmd_ssl(command, response_buffer);
		}
		default:
		{
			append_bad_request(response_buffer);
			return COMMAND_EXECUTED_SUCCESSFULLY;
		}
	}
}
