const ROOT_URL = window.location.href       // for production code
// const ROOT_URL = "http://localhost:3000/"   // for testing with local json server
let conn_status = 0;		// connection status to the ESP32
let old_conn_status = 0;	// connection status before last update of UI to know when it changed
let serial_via_JTAG = 0;	// set to 1 if ESP32 is using the USB interface as serial interface for data and not using the UART. If 0 we set UART config to invisible for the user.
let last_byte_count = 0;
let last_timestamp_byte_count = 0;
let esp_chip_model = 0;		// according to get_esp_chip_model_str()

function change_radio_dis_arm_visibility() {
	// we only support this feature when MAVLink or LTM are set AND when a standard Wi-Fi mode is enabled
	let radio_dis_onarm_div = document.getElementById("radio_dis_onarm_div")
	if (document.getElementById("esp32_mode").value > "2" || document.getElementById("proto").value === "5") {
		radio_dis_onarm_div.style.display = "none";
	} else {
		radio_dis_onarm_div.style.display = "block";
	}
}

function change_ap_ip_visibility(){
	let ap_ip_div = document.getElementById("ap_ip_div");
	let ap_channel_div = document.getElementById("ap_channel_div");
	let disclamer_div = document.getElementById("esp-lr-ap-disclaimer");
	let wifi_ssid_div = document.getElementById("wifi_ssid_div");
	let wifi_en_gn_div = document.getElementById("wifi_en_gn_div");
	let static_ip_config_div = document.getElementById("static_ip_config_div");
	if (document.getElementById("esp32_mode").value === "2") {
		ap_ip_div.style.display = "none";
		ap_channel_div.style.display = "none";
		wifi_en_gn_div.style.display = "block";
		static_ip_config_div.style.display = "block";
	} else {
		ap_ip_div.style.display = "block";
		ap_channel_div.style.display = "block";
		wifi_en_gn_div.style.display = "none";
		static_ip_config_div.style.display = "none";
	}
	if (document.getElementById("esp32_mode").value > "2") {
		disclamer_div.style.display = "block";
	} else {
		disclamer_div.style.display = "none";
	}
	if (document.getElementById("esp32_mode").value > "3") {
		ap_ip_div.style.display = "none";
		wifi_ssid_div.style.visibility = 'hidden';
	} else {
		wifi_ssid_div.style.visibility = "visible";
	}
	change_radio_dis_arm_visibility();
}

function change_msp_ltm_visibility(){
	let msp_ltm_div = document.getElementById("msp_ltm_div");
	let trans_pack_size_div = document.getElementById("trans_pack_size_div");
	let rep_rssi_dbm_div = document.getElementById("rep_rssi_dbm_div");
	let telem_proto = document.getElementById("proto");
	if (telem_proto.value === "1") {
		msp_ltm_div.style.display = "block";
		trans_pack_size_div.style.display = "none";

	} else {
		msp_ltm_div.style.display = "none";
		trans_pack_size_div.style.display = "block";
	}
	if (telem_proto.value === "4") {
		rep_rssi_dbm_div.style.display = "block";
	} else {
		rep_rssi_dbm_div.style.display = "none";
	}
	change_radio_dis_arm_visibility();
}

function change_uart_visibility() {
	let tx_rx_div = document.getElementById("tx_rx_div");
	let rts_cts_div = document.getElementById("rts_cts_div");
	let rts_thresh_div = document.getElementById("rts_thresh_div");
	let baud_div = document.getElementById("baud_div");
	if (serial_via_JTAG === 0) {
		rts_cts_div.style.display = "block";
		tx_rx_div.style.display = "block";
		rts_thresh_div.style.display = "block";
		baud_div.style.display = "block";
	} else {
		rts_cts_div.style.display = "none";
		tx_rx_div.style.display = "none";
		rts_thresh_div.style.display = "none";
		baud_div.style.display = "none";
	}
}

function change_led_config_visibility() {
    let led_config_div = document.getElementById("led_config_div");
    let led_enable_checkbox = document.getElementById("led_enable");
    if (led_enable_checkbox.checked) {
        led_config_div.style.display = "block";
    } else {
        led_config_div.style.display = "none";
    }
}

function flow_control_check() {
	let gpio_rts = document.getElementById("gpio_rts");
	let gpio_cts = document.getElementById("gpio_cts");
	if (isNaN(gpio_rts.value) || isNaN(gpio_cts.value) || gpio_cts.value === '' || gpio_rts.value === '' || gpio_rts.value === gpio_cts.value) {
		show_toast("UART flow control disabled.")
	} else {
		show_toast("UART flow control enabled. Make sure RTS & CTS pins are connected!");
	}
}

/**
 * Convert a form into a JSON string
 * @param form The HTML form to convert
 * @returns {string} JSON formatted string
 */
function toJSONString(form) {
	let obj = {}
	let elements = form.querySelectorAll("input, select")
	for (let i = 0; i < elements.length; ++i) {
		let element = elements[i]
		let name = element.name;
		let value = element.value;
		// parse numbers as numbers except for the SSID and the password fields
		if (!isNaN(Number(value)) && (name.localeCompare("wifi_ssid") !== 0) && (name.localeCompare("wifi_pass") !== 0)) {
			if (name) {
				obj[name] = parseInt(value)
			}
		} else {
			if (name) {
				if (element.type === "checkbox") {
					// convert checked/not checked to 1 & 0 as value
					obj[name] = element.checked ? 1 : 0;
				} else {
					// just get the value specified by the input/select
					obj[name] = value
				}
			}
		}
	}
	return JSON.stringify(obj)
}

/**
 * Request data from the ESP to display in the GUI
 * @param api_path API path/request path
 * @returns {Promise<any>}
 */
async function get_json(api_path) {
	let req_url = ROOT_URL + api_path;

	const controller = new AbortController()
	// Set a timeout limit for the request using `setTimeout`. If the body
	// of this timeout is reached before the request is completed, it will
	// be cancelled.

	const timeout = setTimeout(() => {
		controller.abort()
	}, 2000) // Increased timeout to 2000ms
	const response = await fetch(req_url, {
		signal: controller.signal
	});
	if (!response.ok) {
		const message = `An error has occured: ${response.status}`;
		conn_status = 0
		throw new Error(message);
	}
	return await response.json();
}

/**
 * Create a response with JSON data attached
 * @param api_path API URL path
 * @param json_data JSON body data to send
 * @returns {Promise<any>}
 */
async function send_json(api_path, json_data = undefined) {
	let post_url = ROOT_URL + api_path;
	const response = await fetch(post_url, {
		method: 'POST',
		headers: {
			'Accept': 'application/json',
			'Content-Type': 'application/json',
			"charset": 'utf-8'
		},
		body: json_data
	});
	if (!response.ok) {
		conn_status = 0
		const message = `An error has occured: ${response.status}`;
		throw new Error(message);
	}
	return await response.json();
}

function get_esp_chip_model_str(esp_model_index) {
	switch (esp_model_index) {
		default:
		case 0:
			return "unknown/unsupported ESP32 chip";
		case 1:
			return "ESP32";
		case 2:
			return "ESP32-S2";
		case 9:
			return "ESP32-S3";
		case 5:
			return "ESP32-C3";
		case 13:
			return "ESP32-C6";
		case 12:
			return "ESP32-C5";
	}
}

function get_system_info() {
	get_json("api/system/info").then(json_data => {
		console.log("Received settings: " + json_data)
		document.getElementById("about").innerHTML = "DroneBridge for ESP32 v" + json_data["major_version"] +
			"." + json_data["minor_version"] + "." + json_data["patch_version"] + " ("+json_data["maturity_version"]+")" +
			" - esp-idf " + json_data["idf_version"] + " - " + get_esp_chip_model_str(json_data["esp_chip_model"])
		document.getElementById("esp_mac").innerHTML = json_data["esp_mac"]
		serial_via_JTAG = json_data["serial_via_JTAG"];
		// set external antenna option visible based on info if RF switch is available on the board
		if (parseInt(json_data["has_rf_switch"]) === 1) {
			document.getElementById("ant_use_ext_div").style.display = "block";
		} else {
			document.getElementById("ant_use_ext_div").style.display = "none";
		}
	}).catch(error => {
		conn_status = 0
		error.message;
		return -1;
	});
	return 0;
}

function update_conn_status() {
	if (conn_status)
		document.getElementById("web_conn_status").innerHTML = "<span class=\"dot_green\"></span> connected to ESP32"
	else {
		document.getElementById("web_conn_status").innerHTML = "<span class=\"dot_red\"></span> disconnected from ESP32"
		document.getElementById("current_client_ip").innerHTML = ""
	}
	if (conn_status !== old_conn_status) {
		// connection status changed. Update settings and UI
		get_system_info();
		get_settings();
		setTimeout(change_msp_ltm_visibility, 500);
		setTimeout(change_ap_ip_visibility, 500);
		setTimeout(change_uart_visibility, 500);
	}
	old_conn_status = conn_status
}

/**
 * Get connection status information and display it in the GUI
 */
function get_stats() {
	get_json("api/system/stats").then(json_data => {
		conn_status = 1
		let d = new Date();
		let bytes = parseInt(json_data["read_bytes"])
		let bytes_per_second = 0;
		let current_time = d.getTime();
		if (last_byte_count > 0 && last_timestamp_byte_count > 0 && !isNaN(bytes)) {
			bytes_per_second = (bytes - last_byte_count) / ((current_time - last_timestamp_byte_count) / 1000);
		}
		last_timestamp_byte_count = current_time;
		if (!isNaN(bytes) && bytes > 1000000) {
			document.getElementById("read_bytes").innerHTML = (bytes / 1000000).toFixed(3) + " MB (" + ((bytes_per_second*8)/1000).toFixed(2) + " kbit/s)"
		} else if (!isNaN(bytes) && bytes > 1000) {
			document.getElementById("read_bytes").innerHTML = (bytes / 1000).toFixed(2) + " kB (" + ((bytes_per_second*8)/1000).toFixed(2) + " kbit/s)"
		} else if (!isNaN(bytes)) {
			document.getElementById("read_bytes").innerHTML = bytes + " bytes (" + Math.round(bytes_per_second) + " byte/s)"
		}
		last_byte_count = bytes;

		let tcp_clients = parseInt(json_data["tcp_connected"])
		if (!isNaN(tcp_clients) && tcp_clients === 1) {
			document.getElementById("tcp_connected").innerHTML = tcp_clients + " client"
		} else if (!isNaN(tcp_clients)) {
			document.getElementById("tcp_connected").innerHTML = tcp_clients + " clients"
		}
		// UDP clients for tooltip
		let udp_clients_string = ""
		if (json_data.hasOwnProperty("udp_clients")) {
			let udp_conn_jsonarray = json_data["udp_clients"];
			for (let i = 0; i < udp_conn_jsonarray.length; i++) {
				udp_clients_string = udp_clients_string + udp_conn_jsonarray[i];
				if ((i + 1) !== udp_conn_jsonarray.length) {
					udp_clients_string = udp_clients_string + "<br>";
				}
			}
			if (udp_conn_jsonarray.length === 0) {
				udp_clients_string = "-";
			} else {
				document.getElementById("tooltip_udp_clients").innerHTML = udp_clients_string;
			}
		}

		let udp_clients = parseInt(json_data["udp_connected"])
		if (!isNaN(udp_clients) && udp_clients === 1) {
			document.getElementById("udp_connected").innerHTML = "<span class=\"tooltiptext\" id=\"tooltip_udp_clients\">"+udp_clients_string+"</span>" + udp_clients + " client"
		} else if (!isNaN(udp_clients)) {
			document.getElementById("udp_connected").innerHTML = "<span class=\"tooltiptext\" id=\"tooltip_udp_clients\">"+udp_clients_string+"</span>" + udp_clients + " clients"
		}

		if ('esp_rssi' in json_data) {
			let rssi = parseInt(json_data["esp_rssi"])
			if (!isNaN(rssi) && rssi < 0) {
				document.getElementById("current_client_ip").innerHTML = "IP Address: " + json_data["current_client_ip"] + "<br />Signal Strength: " + rssi + "dBm"
			} else if (!isNaN(rssi)) {
				document.getElementById("current_client_ip").innerHTML = "IP Address: " + json_data["current_client_ip"]
			}
		} else if ('connected_sta' in json_data) {
			let a = ""
			json_data["connected_sta"].forEach((item) => {
				a = a + "Client: " + item.sta_mac + " Signal Strength: " + item.sta_rssi + "dBm<br />"
			});
			document.getElementById("current_client_ip").innerHTML = a
		}

	}).catch(error => {
		conn_status = 0
		error.message;
	});
}

/**
 * Get settings from ESP and display them in the GUI. JSON objects have to match the element ids
 *  returns 0 on success and -1 on failure
 */
function get_settings() {
	get_json("api/settings").then(json_data => {
		console.log("Received settings: " + json_data)
		conn_status = 1
		for (const key in json_data) {
			if (json_data.hasOwnProperty(key)) {
				let elem = document.getElementById(key)
				if (elem != null) {
					if (elem.type === "checkbox") {
						// translate 1 & 0 to checked and not checked
						elem.checked = json_data[key] === 1;
					} else {
						elem.value = json_data[key] + ""
					}
				}
			}
		}
	}).catch(error => {
		conn_status = 0
		error.message;
		show_toast(error.message);
		return -1;
	});
	change_ap_ip_visibility();
	change_msp_ltm_visibility();
	return 0;
}

function add_new_udp_client() {
	let ip = prompt("Please enter the IP address of the UDP receiver", "192.168.2.X");
	let port = prompt("Please enter the port number of the UDP receiver", "14550");
	port = parseInt(port);
	let save_to_nvm = confirm("Save this UDP client to the permanent storage so it will be auto added after reboot/reset?\nYou can only save one UDP client to the memory. The old ones will be overwritten.\nSelect no if you only want to add this client for this session.");
	const ippattern = /^(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$/;

	if (ip != null && port != null && ippattern.test(ip)) {
		let myjson = {
			udp_client_ip: ip,
			udp_client_port: port,
			save: save_to_nvm
		};
		send_json("api/settings/clients/udp", JSON.stringify(myjson)).then(send_response => {
			console.log(send_response);
			conn_status = 1
			show_toast(send_response["msg"])
		}).catch(error => {
			show_toast(error.message);
		});
	} else {
		show_toast("Error: Enter valid IP and port!")
	}
}

async function clear_udp_clients() {
	if (confirm("Do you want to remove all UDP connections?\nGCS will have to re-connect.") === true) {
		let post_url = ROOT_URL + "api/settings/clients/clear_udp";
		const response = await fetch(post_url, {
			method: 'DELETE',
			headers: {
				'Accept': 'application/json',
				'Content-Type': 'application/json',
				"charset": 'UTF-8'
			},
			body: null
		});
		if (!response.ok) {
			conn_status = 0
			const message = `An error has occured: ${response.status}`;
			throw new Error(message);
		}
	} else {
		// cancel
	}
}

function show_toast(msg, background_color = "#0058a6") {
	Toastify({
		text: msg,
		duration: 5000,
		newWindow: true,
		close: true,
		gravity: "top", // `top` or `bottom`
		position: "center", // `left`, `center` or `right`
		style: {
			background: background_color,
			color: "#ff9734",
			borderColor: "#ff9734",
			borderStyle: "solid",
			borderRadius: "2px",
			borderWidth: "1px",
		},
		stopOnFocus: true, // Prevents dismissing of toast on hover
	}).showToast();
}

function check_validity() {
	let valid = true;
	let wifi_pass = document.getElementById("wifi_pass")
	if (!wifi_pass.checkValidity()) {
		show_toast("Error: 8<(password length)<64");
		valid = false;
	}
	return valid;
}

function save_settings() {
	let form = document.getElementById("settings_form")
	if (check_validity()) {
		let json_data = toJSONString(form)
		send_json("api/settings", json_data).then(send_response => {
			console.log(send_response);
			conn_status = 1
			show_toast(send_response["msg"])
			get_settings()  // update UI with new settings
		}).catch(error => {
			show_toast(error.message);
		});
	} else {
		console.log("Form was not filled out correctly.")
	}
}

/**
 * Sends a request to the ESP32 to test the LED strip.
 */
function test_led_strip() {
    if (!document.getElementById("led_enable").checked) {
        show_toast("LED strip is not enabled in settings.", "#ffcc00");
        return;
    }
    get_json("api/led/test").then(response => {
        console.log(response);
        conn_status = 1;
        show_toast(response["msg"]);
    }).catch(error => {
        conn_status = 0;
        show_toast("Error starting LED test: " + error.message, "#ff0000");
    });
}
