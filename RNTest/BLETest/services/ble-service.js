import {BleManager} from "react-native-ble-plx";
import {PermissionsAndroid} from 'react-native';


const SERVICE_UUID = '0000fff0-0000-1000-8000-00805f9b34fb';
const CHAR_UUID = '0000fff4-0000-1000-8000-00805f9b34fb';


export default class BLE{
    static BLEManager = new BleManager();
    static BLEDevice = null;
    static DeviceSub = null;

    static scanForDevice = async (callback) => {
        await PermissionsAndroid.request(
            PermissionsAndroid.PERMISSIONS.ACCESS_COARSE_LOCATION,
        );
        await PermissionsAndroid.request(
            PermissionsAndroid.PERMISSIONS.ACCESS_FINE_LOCATION,
        );
        await PermissionsAndroid.request(
            PermissionsAndroid.PERMISSIONS.SEND_SMS
        );

        this.BLEManager.startDeviceScan(null, null, callback);
    };

    static stopScan = () => {
        this.BLEManager.stopDeviceScan();
    };

    static connectAndMonitorDevice = async (device, callback) => {
        let connectedDevice = await device.connect();
        let discoveredDevice = await connectedDevice.discoverAllServicesAndCharacteristics();
        this.BLEDevice = discoveredDevice;
        this.DeviceSub = discoveredDevice.monitorCharacteristicForService(SERVICE_UUID, CHAR_UUID, callback);
    }


}
