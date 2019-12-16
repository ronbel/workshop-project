
import React, {useState} from 'react';
import {
    SafeAreaView,
    Button,
    StatusBar,
    Text,
    Alert,
    PermissionsAndroid,
} from 'react-native';
import {BleManager} from 'react-native-ble-plx';

const BLEManager = new BleManager();

const charUUID = '0000fff1-0000-1000-8000-00805f9b34fb';



const App: () => React$Node = () => {
    const [foundMyDevice, setFoundMyDevice] = useState(false);
    const [char, setChar] = useState('');


    const addDevice = (err, device) => {
        if (err) {
            Alert.alert('Error', err.message);
        }
        if (device && device.name && device.name.toLowerCase() === 'lrrm') {
            setFoundMyDevice(true);

            device.connect().then(async device => {
                let fullDevice = await device.discoverAllServicesAndCharacteristics();
                let services = await fullDevice.services();
                for( let service of services){
                    let chars = await service.characteristics();

                    if(chars.some( x => x.uuid === charUUID)){
                        console.warn('Found char!')
                        let myChar = chars.find(x => x.uuid === charUUID)
                        myChar = await myChar.read();
                        setChar(myChar.value);
                        setInterval(async () => {
                            console.warn('Reading char again')
                            let yayChar = await myChar.read();
                            setChar(yayChar.value)
                        },1000);
                        console.warn('Started monitoring char')
                    }


                }
            }).catch(err => Alert.alert('Error', err.message));
        }
        BLEManager.stopDeviceScan();
    };

    const startScan = async () => {
        await PermissionsAndroid.request(
            PermissionsAndroid.PERMISSIONS.ACCESS_COARSE_LOCATION,
        );
        await PermissionsAndroid.request(
            PermissionsAndroid.PERMISSIONS.ACCESS_FINE_LOCATION,
        );
        BLEManager.startDeviceScan(null, null, addDevice);
    };
    return (
        <>
            <StatusBar barStyle="dark-content"/>
            <SafeAreaView
                style={{flex: 1, alignItems: 'center', justifyContent: 'center'}}>
                <Button title={'Scan'} onPress={startScan}/>
                {foundMyDevice && <Text>Yay, I found LRRM</Text>}
                <Text>Tracked char value is {char}</Text>
            </SafeAreaView>
        </>
    );
};

export default App;
