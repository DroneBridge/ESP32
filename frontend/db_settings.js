// const ROOT_URL = "http://localhost:3000/"   // for testing with local json server
const ROOT_URL = window.location.href       // for production code
let conn_status = 0;

function change_ap_ip_visibility(){
    var ap_ip_div = document.getElementById("ap_ip_div");
    var ap_channel_div = document.getElementById("ap_channel_div");
    if (document.getElementById("esp32_mode").value < 2) {
        ap_ip_div.style.display = "block";
        ap_channel_div.style.display = "block";
    } else {
        ap_ip_div.style.display = "none";
        ap_channel_div.style.display = "none";
    }
}

function change_msp_ltm_visibility(){
    var msp_ltm_div = document.getElementById("msp_ltm_div");
    var trans_pack_size_div = document.getElementById("trans_pack_size_div");
    if (document.getElementById("telem_proto").value === "1") {
        msp_ltm_div.style.display = "block";
        trans_pack_size_div.style.display = "none";
    } else {
        msp_ltm_div.style.display = "none";
        trans_pack_size_div.style.display = "block";
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
        if (!isNaN(Number(value))) {
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
async function send_json(api_path, json_data) {
    let post_url = ROOT_URL + api_path;
    const response = await fetch(post_url, {
        method: 'POST',
        headers: {
            'Accept': 'application/json',
            'Content-Type': 'application/json'
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
    }).catch(error => {
        conn_status = 0
        error.message;
    });
}

function update_conn_status() {
    if (conn_status)
        document.getElementById("web_conn_status").innerHTML = "<span class=\"dot_green\"></span> connected to ESP32"
    else {
        document.getElementById("web_conn_status").innerHTML = "<span class=\"dot_red\"></span> disconnected from ESP32"
        document.getElementById("current_client_ip").innerHTML = ""
    }
}

/**
 * Get connection status information and display it in the GUI
 */
function get_stats() {
    get_json("api/system/stats").then(json_data => {
        conn_status = 1
        let bytes = parseInt(json_data["read_bytes"])
        if (!isNaN(bytes) && bytes > 1000) {
            document.getElementById("read_bytes").innerHTML = (bytes / 1000) + " kb"
        } else if (!isNaN(bytes)) {
            document.getElementById("read_bytes").innerHTML = bytes + " bytes"
        }

        let tcp_clients = parseInt(json_data["tcp_connected"])
        if (!isNaN(tcp_clients) && tcp_clients > 1) {
            document.getElementById("tcp_connected").innerHTML = tcp_clients + " clients"
        } else if (!isNaN(tcp_clients)) {
            document.getElementById("tcp_connected").innerHTML = tcp_clients + " client"
        }

        let udp_clients = parseInt(json_data["udp_connected"])
        if (!isNaN(udp_clients) && udp_clients > 1) {
            document.getElementById("udp_connected").innerHTML = udp_clients + " clients"
        } else if (!isNaN(udp_clients)) {
            document.getElementById("udp_connected").innerHTML = udp_clients + " client"
        }

        document.getElementById("current_client_ip").innerHTML = "IP Address: " + json_data["current_client_ip"]
    }).catch(error => {
        conn_status = 0
        error.message;
    });
}

/**
 * Get settings from ESP and display them in the GUI. JSON objects have to match the element ids
 */
function get_settings() {
    get_json("api/settings/request").then(json_data => {
        console.log("Received settings: " + json_data)
        conn_status = 1
        for (const key in json_data) {
            if (json_data.hasOwnProperty(key)) {
                let elem = document.getElementById(key)
                elem.value = json_data[key] + ""
            }
        }
    }).catch(error => {
        conn_status = 0
        error.message;
    });
    change_ap_ip_visibility();
    change_msp_ltm_visibility();
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
    send_json("api/settings/change", json_data).then(send_response => {
        console.log(send_response);
        conn_status = 1
        show_toast(send_response["msg"])
        get_settings()  // update UI with new settings
    });
}

function trigger_reboot() {
    get_json("api/system/reboot").then(json_data => {
        show_toast(json_data["msg"])
    }).catch(error => {
        error.message;
    });
}