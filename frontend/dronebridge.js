// const ROOT_URL = "http://localhost:3000/"   // for testing with local json server
const ROOT_URL = window.location.href       // for production code
let conn_status = 0;		// connection status to the ESP32
let old_conn_status = 0;	// connection status before last update of UI to know when it changed
let serial_via_JTAG = 0;	// set to 1 if ESP32 is using the USB interface as serial interface for data and not using the UART. If 0 we set UART config to invisible for the user.
let last_byte_count = 0;
let last_timestamp_byte_count = 0;

function change_ap_ip_visibility(){
	let ap_ip_div = document.getElementById("ap_ip_div");
	let ap_channel_div = document.getElementById("ap_channel_div");
	let disclamer_div = document.getElementById("esp-lr-ap-disclaimer");
	let wifi_ssid_div = document.getElementById("wifi_ssid_div");
	if (document.getElementById("esp32_mode").value === "2") {
		ap_ip_div.style.display = "none";
		ap_channel_div.style.display = "none";
	} else {
		ap_ip_div.style.display = "block";
		ap_channel_div.style.display = "block";
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
}

function change_msp_ltm_visibility(){
	let msp_ltm_div = document.getElementById("msp_ltm_div");
	let trans_pack_size_div = document.getElementById("trans_pack_size_div");
	if (document.getElementById("telem_proto").value === "1") {
		msp_ltm_div.style.display = "block";
		trans_pack_size_div.style.display = "none";
	} else {
		msp_ltm_div.style.display = "none";
		trans_pack_size_div.style.display = "block";
	}
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
		let name = element.name
		let value = element.value;
		// parse numbers as numbers except for the SSID and the password fields
		if (!isNaN(Number(value)) && (name.localeCompare("wifi_ssid") !== 0) && (name.localeCompare("wifi_pass") !== 0)) {
			if (name) {
				obj[name] = parseInt(value)
			}
		} else {
			if (name) {
				obj[name] = value
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
	}, 1000)
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

function get_system_info() {
	get_json("api/system/info").then(json_data => {
		console.log("Received settings: " + json_data)
		document.getElementById("about").innerHTML = "DroneBridge for ESP32 - v" + json_data["major_version"] +
			"." + json_data["minor_version"] + " - esp-idf " + json_data["idf_version"]
		document.getElementById("esp_mac").innerHTML = json_data["esp_mac"]
		serial_via_JTAG = json_data["serial_via_JTAG"];
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
				document.getElementById("current_client_ip").innerHTML = "IP Address: " + json_data["current_client_ip"] + "<br />RSSI: " + rssi + "dBm"
			} else if (!isNaN(rssi)) {
				document.getElementById("current_client_ip").innerHTML = "IP Address: " + json_data["current_client_ip"]
			}
		} else if ('connected_sta' in json_data) {
			let a = ""
			json_data["connected_sta"].forEach((item) => {
				a = a + "Client: " + item.sta_mac + " RSSI: " + item.sta_rssi + "dBm<br />"
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
					elem.value = json_data[key] + ""
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
			ip: ip,
			port: port,
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

function show_toast(msg) {
	Toastify({
		text: msg,
		duration: 5000,
		newWindow: true,
		close: true,
		gravity: "top", // `top` or `bottom`
		position: "center", // `left`, `center` or `right`
		// style: {
		//     background: "linear-gradient(to right, #00b09b, #96c93d)"
		// },
		backgroundColor: "linear-gradient(to right, #b6e026, #abdc28)",
		stopOnFocus: true, // Prevents dismissing of toast on hover
	}).showToast();
}

function save_settings() {
	let form = document.getElementById("settings_form")
	let json_data = toJSONString(form)
	send_json("api/settings", json_data).then(send_response => {
		console.log(send_response);
		conn_status = 1
		show_toast(send_response["msg"])
		get_settings()  // update UI with new settings
	}).catch(error => {
		show_toast(error.message);
	});
}
