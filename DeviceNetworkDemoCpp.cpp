#include <iostream>

#include <Antilatency.InterfaceContract.LibraryLoader.h>
#include <Antilatency.DeviceNetwork.h>


bool exitRequested = false;

BOOL WINAPI consoleHandler(DWORD signal) {
    if (signal == CTRL_C_EVENT) {
        exitRequested = true;
        return FALSE;
    } else if (signal == CTRL_CLOSE_EVENT || signal == CTRL_BREAK_EVENT) {
        exitRequested = true;
        return FALSE;
    }

    return FALSE;
}

/*
 * Get a device filter for all Antilatency USB devices.
 */
Antilatency::DeviceNetwork::IDeviceFilter getAllUsbDevicesFilter(Antilatency::DeviceNetwork::ILibrary deviceNetworkLibrary) {
    Antilatency::DeviceNetwork::IDeviceFilter result = deviceNetworkLibrary.createFilter();
    result.addUsbDevice(Antilatency::DeviceNetwork::Constants::AllUsbDevices);
    return result;
}

/*
 * Get a device filter for all IP devices.
 */
Antilatency::DeviceNetwork::IDeviceFilter getAllIpDevicesFilter(Antilatency::DeviceNetwork::ILibrary deviceNetworkLibrary) {
    Antilatency::DeviceNetwork::IDeviceFilter result = deviceNetworkLibrary.createFilter();
    result.addIpDevice(Antilatency::DeviceNetwork::Constants::AllIpDevicesIp, Antilatency::DeviceNetwork::Constants::AllIpDevicesMask);
    return result;
}

/*
* Get a device filter for all IP devices.
*/
Antilatency::DeviceNetwork::IDeviceFilter getAntilatencyUsbSocketDevicesFilter(Antilatency::DeviceNetwork::ILibrary deviceNetworkLibrary) {
    Antilatency::DeviceNetwork::IDeviceFilter result = deviceNetworkLibrary.createFilter();

    Antilatency::DeviceNetwork::UsbDeviceFilter usbDeviceFilter;
    usbDeviceFilter.vid = Antilatency::DeviceNetwork::UsbVendorId::Antilatency;
    usbDeviceFilter.pid = 0x0000;
    usbDeviceFilter.pidMask = 0xFFFF;

    result.addUsbDevice(usbDeviceFilter);
    return result;
}

int main() {

#ifdef _WIN64
    SetDllDirectory(L"AntilatencySdk/Bin/Windows/x64");
#else
    SetDllDirectory(L"AntilatencySdk/Bin/Windows/x86");
#endif
    
    // Load the Antilatency Device Network library
    Antilatency::DeviceNetwork::ILibrary deviceNetworkLibrary = Antilatency::InterfaceContract::getLibraryInterface<Antilatency::DeviceNetwork::ILibrary>("AntilatencyDeviceNetwork");
    if (deviceNetworkLibrary == nullptr) {
        std::cout << "Failed to get Antilatency Device Network Library" << std::endl;
        return 1;
    }

    const auto deviceNetworkLibraryVersion = deviceNetworkLibrary.getVersion();

    // Create a device network filter and then create a network using that filter.
    const Antilatency::DeviceNetwork::IDeviceFilter networkFilter = getAllUsbDevicesFilter(deviceNetworkLibrary);
    Antilatency::DeviceNetwork::INetwork network = deviceNetworkLibrary.createNetwork(networkFilter);

    if (network == nullptr) {
        std::cout << "Failed to create Antilatency Device Network" << std::endl;
        return 1;
    }

    std::cout << "Antilatency Device Network created, version: " << deviceNetworkLibraryVersion << std::endl;

    // Each time the device network is changed due to connection or disconnection of a device that matches the device filter of the network,
    // or start or stop of a task on any network device, the network update id is incremented by 1. 
    uint32_t prevUpdateId = 0;

    while (true) {
        if (exitRequested) {
            break;
        }

        // Check if the network has been changed.
        const uint32_t currentUpdateId = network.getUpdateId();
        if (prevUpdateId != currentUpdateId) {
            prevUpdateId = currentUpdateId;
            
            std::cout << "--- Device network changed, update id: " << currentUpdateId << " ---" << std::endl;

            // Get the array of currently connected nodes.
            std::vector<Antilatency::DeviceNetwork::NodeHandle> nodes = network.getNodes();

            // Print some information for each node. Reading the property of a newly connected node starts the property task on that node,
            // so you will see additional increments of update id: first at node connection, second at property task start and third when property task finished.
            for (auto node : nodes) {
                // Get some property values for a node.
                std::string hardwareName = network.nodeGetStringProperty(node, Antilatency::DeviceNetwork::Interop::Constants::HardwareNameKey);
                std::string hardwareVersion = network.nodeGetStringProperty(node, Antilatency::DeviceNetwork::Interop::Constants::HardwareVersionKey);
                std::string hardwareSerialNo = network.nodeGetStringProperty(node, Antilatency::DeviceNetwork::Interop::Constants::HardwareSerialNumberKey);
                std::string firmwareName = network.nodeGetStringProperty(node, Antilatency::DeviceNetwork::Interop::Constants::FirmwareNameKey);
                std::string firmwareVersion = network.nodeGetStringProperty(node, Antilatency::DeviceNetwork::Interop::Constants::FirmwareVersionKey);

                // Get the node's parent node. When some device (Alt, Tag, Bracer, etc.) is connected to the socket, this socket becomes its parent.
                // A node, that is directly connected to a PC, smartphone, etc. via a USB cable, will have null node handle as its parent.
                Antilatency::DeviceNetwork::NodeHandle parent = network.nodeGetParent(node);
                
                // Get the node status.
                Antilatency::DeviceNetwork::NodeStatus status = network.nodeGetStatus(node);

                std::cout << "Node: " << static_cast<uint32_t>(node) << std::endl;

                std::cout << "\tStatus: " << static_cast<uint32_t>(status) << std::endl;
                std::cout << "\tParent node: " << static_cast<uint32_t>(parent) << std::endl;

                std::cout << "\tProperties: " << std::endl;
                std::cout << "\t\tHardware name: " << hardwareName << std::endl;
                std::cout << "\t\tHardware version: " << hardwareVersion << std::endl;
                std::cout << "\t\tSerial number: " << hardwareSerialNo << std::endl;
                std::cout << "\t\tFirmware name: " << firmwareName << std::endl;
                std::cout << "\t\tFirmware version: " << firmwareVersion << std::endl << std::endl;
            }
        }
        Yield();
    }
}

