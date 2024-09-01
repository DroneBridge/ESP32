// import { ESPLoader, Transport } from './ESP32/esptool/bundle.js';
// import { serial } from './ESP32/web-serial-polyfill/serial.js';
import { ESPLoader, Transport } from './esptool/bundle.js';
import { serial } from './web-serial-polyfill/serial.js';
if (!navigator.serial && navigator.usb) navigator.serial = serial;  // switch to WebSerial over WebUSB by polyfill

const not_compatible_warning = document.getElementById('not_compatible_warning');
const conn_status_label = document.getElementById('conn_status');
const connectButton = document.getElementById("connectButton");
const software_selection = document.getElementById("software_selection");
const hw_selection = document.getElementById("board_selection");
const flash_button_div = document.getElementById("flash_button_div");
const flash_button = document.getElementById("flash_button");
const software_version_selector = document.getElementById("software_version_selector");
const flavor_selector = document.getElementById("flavor_selector");
const loader_div = document.getElementById("loader_div");
const progress_bar1 = document.getElementById("progress_bar1");
const progress_div = document.getElementById("progress_div");
const flashing_div = document.getElementById("flashing_div");
const success_msg_div = document.getElementById("success_msg_div");
const flash_curr_file = document.getElementById("flash_curr_file");

class DBTarget {
    constructor(chip_name, target_display_name, target_folder_name, target_files, target_addresses, target_flash_mode,
                target_flash_freq) {
        this.chip_name = chip_name;
        this.target_display_name = target_display_name;
        this.target_folder_name = target_folder_name;
        this.target_files = target_files;
        this.target_file_addresses = target_addresses;
        this.flash_mode = target_flash_mode;
        this.flash_freq = target_flash_freq;
    }
}

class DBRelease {
    constructor(display_name, folder_name, db_targets) {
        this.display_name = display_name;
        this.folder_name = folder_name;
        this.targets = db_targets;
    }
}

// const terminal = document.getElementById("terminal");
// let Terminal; // Terminal is imported in HTML script
// const term = new Terminal({ cols: 120, rows: 40 });
// term.open(terminal);
// const espLoaderTerminal = {
//     clean() {
//         term.clear();
//     },
//     writeLine(data) {
//         term.writeln(data);
//     },
//     write(data) {
//         term.write(data);
//     },
// };

// Define DroneBridge for ESP32 releases manually here. No GitHub call etc. All locally available
let t_esp32 = new DBTarget("ESP32", "ESP32", "esp32/", ["bootloader.bin", "partition-table.bin", "db_esp32.bin", "www.bin"], [0x1000, 0x8000, 0x10000, 0x110000], "DIO", "40MHz");
let t_espc3 = new DBTarget("ESP32-C3", "ESP32-C3", "esp32c3/", ["bootloader.bin", "partition-table.bin", "db_esp32.bin", "www.bin"], [0x0, 0x8000, 0x10000, 0x110000], "DIO", "80MHz");
let t_esps2 = new DBTarget("ESP32-S2", "ESP32-S2", "esp32s2/", ["bootloader.bin", "partition-table.bin", "db_esp32.bin", "www.bin"], [0x1000, 0x8000, 0x10000, 0x110000], "DIO", "80MHz");
let t_esp32s3 = new DBTarget("ESP32-S3", "ESP32-S3", "esp32s3/", ["bootloader.bin", "partition-table.bin", "db_esp32.bin", "www.bin"], [0x0, 0x8000, 0x10000, 0x110000], "DIO", "80MHz");
let release_15 = new DBRelease("v1.5 (stable)", "./db_releases/1_5/",
    [t_esp32, t_espc3, t_esps2, t_esp32s3]);

let t2RC2_esp32 = new DBTarget("ESP32", "ESP32", "esp32/", ["bootloader.bin", "partition-table.bin", "db_esp32.bin", "www.bin"], [0x1000, 0x8000, 0x10000, 0x190000], "DIO", "40MHz");
let t2RC2_espc3 = new DBTarget("ESP32-C3", "ESP32-C3", "esp32c3/", ["bootloader.bin", "partition-table.bin", "db_esp32.bin", "www.bin"], [0x0, 0x8000, 0x10000, 0x190000], "DIO", "80MHz");
let t2RC2_espc3_usbserial = new DBTarget("ESP32-C3", "ESP32-C3 (USBSerial)", "esp32c3_USBSerial/", ["bootloader.bin", "partition-table.bin", "db_esp32.bin", "www.bin"], [0x0, 0x8000, 0x10000, 0x190000], "DIO", "80MHz");
let t2RC2_espc6 = new DBTarget("ESP32-C6", "ESP32-C6", "esp32c6/", ["bootloader.bin", "partition-table.bin", "db_esp32.bin", "www.bin"], [0x0, 0x8000, 0x10000, 0x190000], "DIO", "80MHz");
let t2RC2_esps2 = new DBTarget("ESP32-S2", "ESP32-S2", "esp32s2/", ["bootloader.bin", "partition-table.bin", "db_esp32.bin", "www.bin"], [0x1000, 0x8000, 0x10000, 0x190000], "DIO", "80MHz");
let t2RC2_esp32s3 = new DBTarget("ESP32-S3", "ESP32-S3", "esp32s3/", ["bootloader.bin", "partition-table.bin", "db_esp32.bin", "www.bin"], [0x0, 0x8000, 0x10000, 0x190000], "DIO", "80MHz");
let release_20RC2 = new DBRelease("v2.0RC2 (pre-release)", "./db_releases/2_0RC2/",
    [t2RC2_esp32, t2RC2_espc3, t2RC2_espc3_usbserial, t2RC2_espc6, t2RC2_esps2, t2RC2_esp32s3]);

let t2RC3_esp32 = new DBTarget("ESP32", "ESP32", "esp32/", ["bootloader.bin", "partition-table.bin", "db_esp32.bin", "www.bin"], [0x1000, 0x8000, 0x10000, 0x190000], "DIO", "40MHz");
let t2RC3_espc3 = new DBTarget("ESP32-C3", "ESP32-C3", "esp32c3/", ["bootloader.bin", "partition-table.bin", "db_esp32.bin", "www.bin"], [0x0, 0x8000, 0x10000, 0x190000], "DIO", "80MHz");
let t2RC3_espc3_usbserial = new DBTarget("ESP32-C3", "ESP32-C3 (USBSerial)", "esp32c3_USBSerial/", ["bootloader.bin", "partition-table.bin", "db_esp32.bin", "www.bin"], [0x0, 0x8000, 0x10000, 0x190000], "DIO", "80MHz");
let t2RC3_espc6 = new DBTarget("ESP32-C6", "ESP32-C6", "esp32c6/", ["bootloader.bin", "partition-table.bin", "db_esp32.bin", "www.bin"], [0x0, 0x8000, 0x10000, 0x190000], "DIO", "80MHz");
let t2RC3_esps2 = new DBTarget("ESP32-S2", "ESP32-S2", "esp32s2/", ["bootloader.bin", "partition-table.bin", "db_esp32.bin", "www.bin"], [0x1000, 0x8000, 0x10000, 0x190000], "DIO", "80MHz");
let t2RC3_esp32s3 = new DBTarget("ESP32-S3", "ESP32-S3", "esp32s3/", ["bootloader.bin", "partition-table.bin", "db_esp32.bin", "www.bin"], [0x0, 0x8000, 0x10000, 0x190000], "DIO", "80MHz");
let release_20RC3 = new DBRelease("v2.0RC3 (pre-release)", "./db_releases/2_0RC3/",
    [t2RC3_esp32, t2RC3_espc3, t2RC3_espc3_usbserial, t2RC3_espc6, t2RC3_esps2, t2RC3_esp32s3]);

// overall array containing all releases with flashing instructions
let db_releases = [release_20RC3, release_20RC2, release_15];


let device = null;
let transport;
let chip = null;
let esploader;

function browser_comp_check() {
    if (!navigator.serial && !navigator.usb) {
        // the browser is not supporting the required technology for this flasher to run
        console.error("Your browser is not supported! Use a Chrome-Based browser instead!");
        not_compatible_warning.style.display = "block";
        flashing_div.style.display = "none";
    } else {
        not_compatible_warning.style.display = "none";
    }
}

/**
 * Opens dialog for choosing the ESP32 and connects to it. Detects ESP32 chip
 * @returns {Promise<void>}
 */
connectButton.onclick = async () => {
    if (device === null) {
        device = await navigator.serial.requestPort({});
        transport = new Transport(device, true);
    }
    let chip_full_name = "Chip was not detected";
    connectButton.style.display = "none";
    loader_div.style.display = "block";
    try {
        const flashOptions = {
            transport,
            baudrate: 460800,
            //terminal: espLoaderTerminal,
        };
        esploader = new ESPLoader(flashOptions);
        chip_full_name = await esploader.main();
        chip = esploader.chip;
    } catch (e) {
        console.error(e);
        // term.writeln(`Error: ${e.message}`);
        display_ui_not_connected();
    }
    console.log("Settings done for: " + chip_full_name);
    loader_div.style.display = "none";
    display_ui_connected();
    populate_software_versions();
    add_new_falvors(software_version_selector.selectedIndex);
}

/**
 * Reads file (DroneBridge firmware file) from server and returns it
 * @param url
 * @returns {Promise<unknown>}
 */
async function read_file(url) {
    try {
        // Fetch the binary file from the server
        const response = await fetch(url);
        const buffer = await response.arrayBuffer(); // Convert the response to an ArrayBuffer

        // Create a Blob from the ArrayBuffer
        const blob = new Blob([buffer]);

        // Return a promise that resolves with the read file
        return new Promise((resolve, reject) => {
            const reader = new FileReader();
            reader.onload = function(event) {
                resolve(event.target.result); // Resolve the promise with the result
            };
            reader.onerror = function(error) {
                reject(error); // Reject the promise if there's an error
            };
            reader.readAsBinaryString(blob);
            // reader.readAsArrayBuffer(blob);  // create DOMString from binary array
        });
    } catch (error) {
        console.error('Error fetching the binary file:', error);
        throw error; // Re-throw the error to be handled by the caller
    }
}

/**
 * Create an array containing the release binaries and offsets that has the shape that esptool-js expects
 * @param release_folder_name
 * @param target_folder_name
 * @param files_array
 * @param offsets_array
 * @returns {Promise<*[]>}
 */
async function create_file_array(release_folder_name, target_folder_name, files_array, offsets_array){
    let esptool_file_array = [];
    for (let i = 0; i < files_array.length; i++) {
        const url = release_folder_name + target_folder_name + files_array[i];
        await read_file(url).then(result => {
            esptool_file_array.push({ data: result, address: offsets_array[i]});
            }
        );
    }
    return esptool_file_array;
}

/**
 * Read the files for that target and flash them to the ESP32
 * @returns {Promise<void>}
 */
flash_button.onclick = async () => {
    if (device != null) {
        display_ui_flashing();
        let db_rel = db_releases[software_version_selector.selectedIndex];
        // get selected flavour based on display name from the overall target list
        let target_name_selected = flavor_selector.options[flavor_selector.selectedIndex].text
        let db_target = null;
        for (let i = 0; i < db_rel.targets.length; i++) {
            if (db_rel.targets[i].target_display_name === target_name_selected) {
                db_target = db_rel.targets[i];
                break;
            }
        }
        if (db_target == null) {
            // That should never happen
            console.error("Could not find selected target name in the DroneBridge targets" + target_name_selected);
            return ;
        }
        try {
            let esptool_file_array = await create_file_array(
                db_rel.folder_name,
                db_target.target_folder_name,
                db_target.target_files,
                db_target.target_file_addresses
            );
            const flashOptions = {
                fileArray: esptool_file_array,
                flashSize: "keep",
                flashMode: db_target.flash_mode,
                flashFreq: db_target.flash_freq,
                eraseAll: false,
                compress: true,
                reportProgress: (fileIndex, written, total) => {
                    progress_bar1.value = (written / total) * 100;
                    flash_curr_file.innerText = "Flashing ("+(fileIndex+1)+"/"+esptool_file_array.length+") " + db_target.target_files[fileIndex];
                },
            };
            await esploader.writeFlash(flashOptions);
        } catch (e) {
            console.error(e);
            // term.writeln(`Error: ${e.message}`);
        } finally {
            display_ui_success();
            // simple analytics event
            sa_event("click_flash", { release: db_rel.display_name, chip: db_target.target_display_name });
        }
    } else {
        // we are not connected
        clean_up();
        display_ui_not_connected();
    }
}

/**
 * User selected a new DroneBridge for ESP32 software version. Update the flavor list
 */
software_version_selector.onchange = async (ev) => {
    if (ev.type === 'change') {
        // Clear list
        let i, L = flavor_selector.options.length - 1;
        for(i = L; i >= 0; i--) {
            flavor_selector.remove(i);
        }
        add_new_falvors(software_version_selector.selectedIndex);
        if (flavor_selector.options.length === 0) {
            // Connected ESP32 is not supported by this release
        }
    } // ignore others
}

/**
 * Adds the targets (chips) of that software version that match the connected chip to the list of options
 * @param index
 */
function add_new_falvors(index) {
    let db_target;
    for (db_target of db_releases.at(index).targets) {
        if (db_target.chip_name === chip.CHIP_NAME) {
            // add to option list
            let option = document.createElement("option");
            option.text = db_target.target_display_name;
            flavor_selector.add(option)
        } else {
            // do not add - wrong chip
        }
    }
}

/**
 * Adds a software release versions to the list
 * @param value DBRelease
 * @param index
 * @param array
 */
function add_to_software_version(value, index, array) {
    let option = document.createElement("option");
    console.log(value.display_name)
    option.text = value.display_name;
    software_version_selector.add(option)
}

/**
 * Add all available software release versions to the list
 */
function populate_software_versions() {
    db_releases.forEach(add_to_software_version);
}

function display_ui_connected() {
    conn_status_label.innerHTML = "Connected to " + chip.CHIP_NAME;
    software_selection.style.display = "block";
    hw_selection.style.display = "block";
    flash_button_div.style.display = "block";
    success_msg_div.style.display = "none";
    flashing_div.style.display = "block";
    progress_div.style.display = "none";
}

function display_ui_not_connected() {
    conn_status_label.innerHTML = "Not connected";
    software_selection.style.display = "none";
    hw_selection.style.display = "none";
    connectButton.style.display = "block";
    flash_button_div.style.display = "none";
    success_msg_div.style.display = "none";
    flashing_div.style.display = "block";
    progress_div.style.display = "none";
}

function display_ui_flashing() {
    success_msg_div.style.display = "none";
    progress_div.style.display = "block";
    flashing_div.style.display = "block";
    flash_button_div.style.display = "none";
}

function display_ui_success() {
    success_msg_div.style.display = "block";
    flashing_div.style.display = "none";
    flash_button_div.style.display = "none";
    progress_div.style.display = "none";
}

function clean_up() {
    display_ui_not_connected();
    device = null;
    transport = null;
    chip = null;
}

browser_comp_check();