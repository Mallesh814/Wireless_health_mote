/*
 * ble113.c
 *
 *  Created on: 20-Mar-2017
 *      Author: mallesh
 */
#include "ble113.h"


int found_devices_count = 0;
bd_addr found_devices[MAX_DEVICES];
bd_addr connect_addr;

enum actions action = action_none;
states state = state_standby;
extern uint32_t debugConsole;


uint8 primary_service_uuid[] = {0x00, 0x28};

uint16 thermometer_handle_start = 0,
       thermometer_handle_end = 0,
       thermometer_handle_measurement = 0,
       thermometer_handle_configuration = 0;


void change_state(states new_state)
{
#ifdef DEBUG
    uint8_t *state_names[state_last] = {
		"standby",
		"advertising",
	    "disconnected",
	    "connecting",
	    "connected",
	    "connected_as_Master",
	    "finding_services",
	    "finding_attributes",
	    "listening_measurements",
	    "finish"
	};

    transfer("DEBUG: State changed: ", debugConsole);
    transfer(state_names[state], debugConsole);
    transfer("--> ", debugConsole);
    transfer(state_names[new_state], debugConsole);
    transfer("\n\r", debugConsole);
#endif
    state = new_state;
}

int cmp_bdaddr(bd_addr first, bd_addr second)
{
    int i;
    for (i = 0; i < sizeof(bd_addr); i++) {
        if (first.addr[i] != second.addr[i]) return 1;
    }
    return 0;
}

void print_bdaddr(bd_addr bdaddr)
{
	uint32_t i = 0;
	uint8_t num[3]="\0";

	for(i = 0; i < 6; i++){
		int_hex_ascii(num, bdaddr.addr[5 - i]);
	    transfer(num, debugConsole);
	    transfer(":", debugConsole);
	}
    transfer("\n\r", debugConsole);

}

void enable_indications(uint8 connection_handle, uint16 client_configuration_handle)
{
    uint8 configuration[] = {0x02, 0x00}; // enable indications
    ble_cmd_attclient_attribute_write(connection_handle, thermometer_handle_configuration, 2, &configuration);
}


void ble_rsp_system_get_info(const struct ble_msg_system_get_info_rsp_t *msg)
{
    uint8_t num[10]="\0";
	transfer("Build: ", debugConsole);

    dec_ascii(num, msg->build);
    transfer(num, debugConsole);
    transfer("\n\r", debugConsole);

    transfer(", protocol_version: ", debugConsole);
	dec_ascii(num, msg->protocol_version);
    transfer(num, debugConsole);
    transfer("\n\r", debugConsole);

    transfer(", hardware: ", debugConsole);


    switch (msg->hw) {
    case 0x01: transfer("BLE112", debugConsole); break;
    case 0x02: transfer("BLED112", debugConsole); break;
    default: transfer("Unknown", debugConsole);
    }
    transfer("\n\r", debugConsole);

    if (action == action_info) change_state(state_finish);
}

void ble_evt_gap_scan_response(const struct ble_msg_gap_scan_response_evt_t *msg)
{
    if (found_devices_count >= MAX_DEVICES) change_state(state_finish);

    int i;
    uint8_t num[10]="\0";
    uint8_t payload[64];
	uint8_t *name = &payload[0];

    // Check if this device already found
    for (i = 0; i < found_devices_count; i++) {
        if (!cmp_bdaddr(msg->sender, found_devices[i])) return;
    }
    found_devices_count++;
    mem_cpy(found_devices[i].addr, msg->sender.addr, sizeof(bd_addr));

    // Parse data
    for (i = 0; i < msg->data.len; ) {
        int8 len = msg->data.data[i++];
        if (!len) continue;
        if (i + len > msg->data.len) break; // not enough data
        uint8 type = msg->data.data[i++];
        switch (type) {
        case 0x09:
//            name = malloc(len);
            mem_cpy(name, msg->data.data + i, len - 1);
            name[len - 1] = '\0';
        }

        i += len - 1;
    }

    print_bdaddr(msg->sender);
    transfer(" RSSI:", debugConsole);

	dec_ascii(num, msg->rssi);
    transfer(num, debugConsole);
    transfer("\n\r", debugConsole);



    transfer(" Name:", debugConsole);
    if (name) transfer(name, debugConsole);
    else transfer("Unknown", debugConsole);
    transfer("\n\r", debugConsole);

//    free(name);
}

void ble_evt_connection_status(const struct ble_msg_connection_status_evt_t *msg)
{
    // New connection
#ifdef DEBUG
    uint8_t num[10]="\0";
    transfer("###\tconnection_status: { \n\r", debugConsole);
    transfer(" connection: ", debugConsole);

	int_hex_ascii(num, msg -> connection);
	transfer(num, debugConsole);

	transfer("\n\r flags: ", debugConsole);
	int_hex_ascii(num, msg -> flags);
	transfer(num, debugConsole);

	transfer("\n\r address: ", debugConsole);
    // this is a "bd_addr" data type, which is a 6-byte uint8_t array
	print_bdaddr(msg -> address);

	transfer("\n\r address_type: ", debugConsole);
	int_hex_ascii(num, msg -> address_type);
	transfer(num, debugConsole);

	transfer("\n\r conn_interval: ", debugConsole);
	int_hex_ascii(num, msg -> conn_interval);
	transfer(num, debugConsole);

    transfer("\n\r timeout: ", debugConsole);
	int_hex_ascii(num, msg -> timeout);
    transfer(num, debugConsole);

    transfer("\n\r latency: ", debugConsole);
	int_hex_ascii(num, msg -> latency);
    transfer(num, debugConsole);

    transfer("\n\r bonding: ", debugConsole);
	int_hex_ascii(num, msg -> bonding);
    transfer(num, debugConsole);
    transfer("\n\r }", debugConsole);
#endif

// "flags" bit description:
//  - bit 0: connection_connected
//           Indicates the connection exists to a remote device.
//  - bit 1: connection_encrypted
//           Indicates the connection is encrypted.
//  - bit 2: connection_completed
//           Indicates that a new connection has been created.
//  - bit 3; connection_parameters_change
//           Indicates that connection parameters have changed, and is set
//           when parameters change due to a link layer operation.

// check for new connection established
	if ((msg -> flags & 0x05) == 0x05) {
		// track state change based on last known state, since we can connect two ways
		if (state == state_advertising) {
			change_state(state_connected);
		} else {
			change_state(state_connected_master);
		}
	}
}

void ble_evt_attclient_group_found(const struct ble_msg_attclient_group_found_evt_t *msg)
{
    if (msg->uuid.len == 0) return;
    uint16 uuid = (msg->uuid.data[1] << 8) | msg->uuid.data[0];

    // First thermometer service found
    transfer("Found Something\n\r", debugConsole);
    if (state == state_finding_services && uuid == THERMOMETER_SERVICE_UUID && thermometer_handle_start == 0) {
        thermometer_handle_start = msg->start;
        thermometer_handle_end = msg->end;
    }
}

void ble_evt_attclient_procedure_completed(const struct ble_msg_attclient_procedure_completed_evt_t *msg)
{
    if (state == state_finding_services) {
        // Thermometer service not found
        if (thermometer_handle_start == 0) {
            transfer("No Health Thermometer service found\n\r", debugConsole);
            change_state(state_finish);
        }
        // Find thermometer service attributes
        else {
            change_state(state_finding_attributes);
            ble_cmd_attclient_find_information(msg->connection, thermometer_handle_start, thermometer_handle_end);
        }
    }
    else if (state == state_finding_attributes) {
        // Client characteristic configuration not found
        if (thermometer_handle_configuration == 0) {
            transfer("No Client Characteristic Configuration found for Health Thermometer service\n\r", debugConsole);
            change_state(state_finish);
        }
        // Enable temperature notifications
        else {
            change_state(state_listening_measurements);
            enable_indications(msg->connection, thermometer_handle_configuration);
        }
    }
}

void ble_evt_attclient_find_information_found(const struct ble_msg_attclient_find_information_found_evt_t *msg)
{
    if (msg->uuid.len == 2) {
        uint16 uuid = (msg->uuid.data[1] << 8) | msg->uuid.data[0];

        if (uuid == THERMOMETER_MEASUREMENT_UUID) {
            thermometer_handle_measurement = msg->chrhandle;
        }
        else if (uuid == THERMOMETER_MEASUREMENT_CONFIG_UUID) {
            thermometer_handle_configuration = msg->chrhandle;
        }
    }
}

#define THERMOMETER_FLAGS_FAHRENHEIT 0x1

void ble_evt_attributes_value(const struct ble_msg_attributes_value_evt_t *msg) {
    #ifdef DEBUG
        uint8_t num[10] = "\0";
		uint8_t i = 0;

		transfer("###\tattributes_value: { \n\r", debugConsole);
        transfer("connection: ", debugConsole);
		int_hex_ascii(num, msg -> connection);
        transfer(num, debugConsole);

        transfer("\n\r reason: ", debugConsole);
		int_hex_ascii(num, msg -> reason);
        transfer(num, debugConsole);

        transfer("\n\r handle: ", debugConsole);
		int_hex_ascii(num, msg -> handle);
        transfer(num, debugConsole);

        transfer("\n\r offset: ", debugConsole);
		int_hex_ascii(num, msg -> offset);
        transfer(num, debugConsole);

        transfer("\n\r value_len: ", debugConsole);
		int_hex_ascii(num, msg -> value.len);
        transfer(num, debugConsole);

        transfer("\n\r value_data: ", debugConsole);

        // this is a "uint8array" data type, which is a length byte and a uint8_t* pointer
        for (i = 0; i < msg -> value.len; i++) {
    		int_hex_ascii(num, msg -> value.data[i]);
            transfer(num, debugConsole);
        }
        transfer("\n\r }", debugConsole);
    #endif

        uint8_t dat[] = "My Name is Mallesh.";

    // check for data written to "c_rx_data" handle
    if (msg -> handle == GATT_HANDLE_C_RX_DATA && msg -> value.len > 0) {
        // set ping 8, 9, and 10 to three lower-most bits of first byte of RX data
        // (nice for controlling RGB LED or something)
		GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1, msg -> value.data[0]);	// Toggle LED0 everytime a key is pressed
		ble_cmd_attributes_write(GATT_HANDLE_C_TX_DATA,0,20,dat);
    }
}
void ble_evt_attclient_attribute_value(const struct ble_msg_attclient_attribute_value_evt_t *msg)
{
    uint8_t num[10]="\0";
	if (msg->value.len < 5) {
        transfer("Not enough fields in Temperature Measurement value", debugConsole);
        change_state(state_finish);
    }

    uint8 flags = msg->value.data[0];
    int8 exponent = msg->value.data[4];
    int mantissa = (msg->value.data[3] << 16) | (msg->value.data[2] << 8) | msg->value.data[1];

    if (exponent >= 0)
        exponent = 0;
    else
        exponent = (exponent < 0) ? (exponent * -1):(exponent) ;

    transfer("Temperature: \n\r", debugConsole);
    transfer("Mantissa: \n\r", debugConsole);
	dec_ascii( num, mantissa );
    transfer(num, debugConsole);
    transfer("\n\r", debugConsole);
    transfer("Exponent: \n\r", debugConsole);
	dec_ascii(num, exponent);
    transfer(num, debugConsole);
    transfer("\n\r", debugConsole);

    if (flags & THERMOMETER_FLAGS_FAHRENHEIT)
        transfer("F", debugConsole);
    else
        transfer("C", debugConsole);
    transfer("\n\r", debugConsole);
}

void ble_evt_connection_disconnected(const struct ble_msg_connection_disconnected_evt_t *msg)
{
	#ifdef DEBUG
        uint8_t num[10]="\0";
		transfer("###\tconnection_disconnect: { \n\r", debugConsole);

		transfer(" connection: ", debugConsole);
		int_hex_ascii(num, msg -> connection);
		transfer(num, debugConsole);

		transfer("\n\r reason: ", debugConsole);
		int_hex_ascii(num, msg -> reason);
		transfer(num, debugConsole);

		transfer("\n\r }", debugConsole);
	#endif

	// set state to DISCONNECTED
	//ble_state = BLE_STATE_DISCONNECTED;
	// ^^^ skip above since we're going right back into advertising below

	// after disconnection, resume advertising as discoverable/connectable
	//ble112.ble_cmd_gap_set_mode(BGLIB_GAP_GENERAL_DISCOVERABLE, BGLIB_GAP_UNDIRECTED_CONNECTABLE);
	//while (ble112.checkActivity(1000));

	// after disconnection, resume advertising as discoverable/connectable (with user-defined advertisement data)
	ble_cmd_gap_set_mode(2, 2);

	// set state to ADVERTISING
	change_state(state_advertising);

}

void output(uint8 len1, uint8* data1, uint16 len2, uint8* data2)
{
	uint8_t dat;
    transfer("Sending : Data\n\r", debugConsole);
	dat = len1+len2;
    if (uart_tx(1, &dat) || uart_tx(len1, data1) || uart_tx(len2, data2)) {
        transfer("ERROR: Writing to serial port failed\n\r", debugConsole);
    }
}

void print_raw_packet(struct ble_header *hdr, unsigned char *data)
{
    uint8_t num[3]="\0";

    transfer("Incoming packet: ", debugConsole);

    int i;

    for (i = 0; i < sizeof(*hdr); i++) {
//        printf("%02x ", ((unsigned char *)hdr)[i]);
    	int_hex_ascii(num, ((unsigned char *)hdr)[i]);
		transfer(num, debugConsole);
		transfer(":", debugConsole);
    }
    for (i = 0; i < hdr->lolen; i++) {
//        printf("%02x ", data[i]);
    	int_hex_ascii(num, data[i]);
		transfer(num, debugConsole);
		transfer(":", debugConsole);
    }
    transfer("\n\r", debugConsole);
}


int read_message(int timeout_ms)
{
    unsigned char data[256]; // enough for BLE
    struct ble_header hdr;
    int r;
    uint8_t num[10]="\0";

    r = uart_rx(sizeof(hdr), (unsigned char *)&hdr, UART_TIMEOUT);
    if (!r) {
        return -1; // timeout
    }
    else if (r < 0) {
        transfer("ERROR: Reading header failed. Error code:", debugConsole);
        dec_ascii(num , r);
        transfer(num, debugConsole);
        transfer("\n\r", debugConsole);
        return 1;
    }

    if (hdr.lolen) {
        r = uart_rx(hdr.lolen, data, UART_TIMEOUT);
        if (r <= 0) {
            transfer("ERROR: Reading header failed. Error code:", debugConsole);
            dec_ascii(num, r);
            transfer(num, debugConsole);
            transfer("\n\r", debugConsole);
            return 1;
        }
    }

    const struct ble_msg *msg = ble_get_msg_hdr(hdr);

#ifdef DEBUG
    print_raw_packet(&hdr, data);
#endif

    if (!msg) {
        transfer("ERROR: Unknown message received\n\r", debugConsole);
        return 1;
//        exit(1);
    }

    msg->handler(data);

    return 0;
}
