function toJSONString(form) {
    let obj = {}
    let elements = form.querySelectorAll("input, select")
    for (let i = 0; i < elements.length; ++i) {
        let element = elements[i]
        let name = element.name
        let value = element.value;
        let parsed = parseInt(value)
        if (!isNaN(parsed) && name.localeCompare("ap_ip")) {
            if (name) {
                obj[name] = parsed
            }
        } else {
            if (name) {
                obj[name] = value
            }
        }
    }
    return JSON.stringify(obj)
}

function get_system_info() {
    let req_url = window.location.href + "api/system/info"
    console.log("Requesting " + req_url);
    const userAction = async () => {
        const response = await fetch(req_url)
        const myJson = await response.json() //extract JSON from the http response
        console.log(myJson)
        document.getElementById("about").value = "DroneBridge for ESP32 - 0." + myJson["db_build_version"] + " - esp-idf " + myJson["idf_version"]
    }
}

function get_stats() {
    let req_url = window.location.href + "api/system/stats"
    console.log("Requesting " + req_url);
    const userAction = async () => {
        const response = await fetch(req_url)
        const myJson = await response.json() //extract JSON from the http response
        console.log(myJson)

        let bytes = parseInt(myJson["read_bytes"])
        if (!isNaN(bytes) && bytes > 1000) {
            document.getElementById("read_bytes").value = (bytes/1000) + " kb"
        } else if (!isNaN(bytes)) {
            document.getElementById("read_bytes").value = bytes + " bytes"
        }

        let tcp_clients = parseInt(myJson["tcp_connected"])
        if (!isNaN(tcp_clients) && tcp_clients > 1) {
            document.getElementById("tcp_connected").value = tcp_clients + " clients"
        } else if (!isNaN(tcp_clients)) {
            document.getElementById("tcp_connected").value = tcp_clients + " client"
        }

        let udp_clients = parseInt(myJson["udp_connected"])
        if (!isNaN(udp_clients) && udp_clients > 1) {
            document.getElementById("udp_connected").value = udp_clients + " clients"
        } else if (!isNaN(udp_clients)) {
            document.getElementById("udp_connected").value = udp_clients + " client"
        }
    }
}

function get_settings() {
    let req_url = window.location.href + "api/settings/request";
    console.log("Requesting " + req_url);
    const userAction = async () => {
        const response = await fetch(req_url)
        const settingsJSON = await response.json() //extract JSON from the http response
        console.log("Received settings: " + settingsJSON)
        // const settingsJSON = JSON.parse(raw_json);
        for (const key in settingsJSON){
            if(settingsJSON.hasOwnProperty(key)){
                let elem = document.getElementById(key)
                elem.value = settingsJSON[key] + ""
            }
        }
    }
    // let raw_json = "{\"wifi_ssid\":\"DroneBridge ESP32343\",\"wifi_pass\":\"dronebridge123\",\"tx_pin\":22,\"rx_pin\":100,\"telem_proto\":4,\"baud\":9600,\"msp_ltm_port\":1,\"trans_pack_size\":64,\"ap_ip\":\"192.168.2.100\"}"
}

function save_settings() {
    let form = document.getElementById("settings_form")
    let json_data = toJSONString(form)
    console.log(json_data)

    let post_url = window.location.href + "api/settings/change";
    const userAction = async () => {
        await fetch(post_url, {
            method: 'POST',
            headers: {
                'Accept': 'application/json',
                'Content-Type': 'application/json'
            },
            body: json_data
        }).then(rawResponse => {
            const content = rawResponse.json();
            rawResponse.status
            console.log(content);
            get_settings()  // update UI with new settings
        });

    }
}