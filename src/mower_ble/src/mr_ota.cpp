/*********************************************************************
 *
 *  This file is part of the [OPEN_MICRO_MOWER_ROS] project.
 *  Licensed under the MIT License for non-commercial purposes.
 *  Author: Brook Li
 *  Email: lguitech@126.com
 *
 *  For more details, refer to the LICENSE file or contact [lguitech@126.com].
 *
 *  Commercial use requires a separate license.
 *
 *  This software is provided "as is", without warranty of any kind.
 *
 *********************************************************************/

#include "mr_ota.h"

namespace fs = boost::filesystem;

mr_class_ota::mr_class_ota()
{
    otaRunning = false;
}

mr_class_ota::~mr_class_ota()
{
    
}
std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe) throw std::runtime_error("popen() failed!");
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}

bool connectToWiFi(const std::string& ssid, const std::string& password) {
    std::string command;
    std::string netId;

    command = "sudo wpa_cli -i wlan0 add_network";
    netId = exec(command.c_str());
    netId = netId.substr(0, netId.find("\n"));

    command = "sudo wpa_cli -i wlan0 set_network " + netId + " ssid '\"" + ssid + "\"'";
    exec(command.c_str());

    command = "sudo wpa_cli -i wlan0 set_network " + netId + " psk '\"" + password + "\"'";
    exec(command.c_str());

    command = "sudo wpa_cli -i wlan0 enable_network " + netId;
    exec(command.c_str());

    std::this_thread::sleep_for(std::chrono::seconds(5));


    command = "sudo wpa_cli -i wlan0 status | grep wpa_state";
    std::string status = exec(command.c_str());

    if (status.find("COMPLETED") != std::string::npos) {

        command = "timeout 30s sudo dhclient wlan0";
        exec(command.c_str());

        command = "ip addr show wlan0 | grep 'inet '";
        std::string ipResult = exec(command.c_str());
        
        if (!ipResult.empty()) {
            std::cout << "Successfully obtained IP address." << std::endl;
            return true;
        } else {
            std::cout << "Failed to obtain IP address." << std::endl;
            return false;
        }
    } else {
        std::cout << "Failed to connect to Wi-Fi. Please check the SSID and password." << std::endl;
        return false;
    }
}


void setDNSServers(const std::vector<std::string>& dnsServers) {
    std::string command = "sudo tee /etc/resolv.conf > /dev/null";
    for (const auto& dns : dnsServers) {
        std::string fullCommand = "echo 'nameserver " + dns + "' | " + command;
        exec(fullCommand.c_str());
    }
}


size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp) {
    std::ofstream* os = static_cast<std::ofstream*>(userp);
    os->write(static_cast<char*>(contents), size * nmemb);
    return size * nmemb;
}


bool DownloadFile(const std::string& url, const std::string& local_path) {
    CURL* curl;
    CURLcode res;
    std::ofstream ofs(local_path, std::ios::binary);
    curl_global_init(CURL_GLOBAL_DEFAULT);
    curl = curl_easy_init();
    if(curl) {
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_PROXY, ""); 
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &ofs);
        curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);
        curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 5L);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 180L);
        res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);
        
        ROS_INFO_STREAM("download finished or timeout");
        ofs.close();
        return res == CURLE_OK;
    }
    ROS_INFO_STREAM("curl is null");
    return false;

}
bool ExtractTarGz(const std::string& tar_gz_file, const std::string& dest_dir) {
    std::string command = "tar xzvf " + tar_gz_file + " -C " + dest_dir;

    int status = system(command.c_str());
    if (status != 0) {
        ROS_INFO_STREAM( "Failed to extract " << tar_gz_file);
        return false;
    }

    return true;
}

int mr_class_ota::threadFunction(mr_class_ota* p_instance)
{
    const std::string server_json_url = "http://xxxx/mower/release_info.json";
    const std::string server_tar_url = "http://xxxx/mower/mower_release.tar.gz";
    const std::string temp_json_path = "/var/tmp/release_info.json";
    const std::string temp_tar_path = "/var/tmp/mower_release.tar.gz";

    const std::string release_dir = std::getenv("HOME") + std::string("/open_micro_mower_ros");

    if (!DownloadFile(server_json_url, temp_json_path)) {
        ROS_INFO_STREAM("Failed to download release_info.json");
        return OTA_RESULT_DOWNLOAD_FAIL;
    }
    ROS_INFO_STREAM("DOWNLOAD FILE FINISHED!");

    if (!DownloadFile(server_tar_url, temp_tar_path)) {
        ROS_INFO_STREAM("Failed to download mower_release.zip");
        return OTA_RESULT_DOWNLOAD_FAIL;
    }

    for (const auto& entry : fs::directory_iterator(release_dir)) {
        fs::remove_all(entry.path());
    }


    ROS_INFO_STREAM("UNZIP FILE DIR = " << release_dir);
    if (ExtractTarGz(temp_tar_path, release_dir)) {
        ROS_INFO_STREAM("Extracted successfully");
    }
    else {
        ROS_INFO_STREAM("Extraction failed");
        return OTA_RESULT_UNZIP_FAIL;
    }

    fs::copy_file(temp_json_path, release_dir + "/release_info.json", fs::copy_option::overwrite_if_exists);

    ROS_INFO_STREAM("Update successful!");
    return OTA_RESULT_OK;
}

int mr_class_ota::do_upgrade(const char* ssid, const char* password)
{
    std::vector<std::string> dnsServers = {"8.8.8.8", "8.8.4.4", "223.5.5.5", "223.6.6.6"};
    if (otaRunning) {
        return OTA_RESULT_INPROGRESS ;
    }
    
    if (!connectToWiFi(ssid, password)) {
        otaRunning = false;
        return OTA_RESULT_NETWORK_FAIL;
    }

    setDNSServers(dnsServers);

    ROS_INFO_STREAM("BEGIN UPGRADE!");

    otaRunning = true;
    int result = mr_class_ota::threadFunction(this);
    otaRunning = false;
    return result;
}