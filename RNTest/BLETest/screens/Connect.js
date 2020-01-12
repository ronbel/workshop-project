import React, {useContext, useEffect, useState} from 'react';
import {ActivityIndicator, Alert, Button, Text, View} from 'react-native';
import BLE from '../services/ble-service';
import {ModalContext} from '../components/TimerModal';


export default function Connect() {
    const [device, setDevice] = useState(null);
    const [found, setFound] = useState(false);
    const [blueError, setBlueError]  = useState(false);
    const [isConnected, setIsConnected] = useState(false);
    const Timer = useContext(ModalContext);

    const addDevice = (err, device) => {
        if(err || !device || !device.name || device.name.toLowerCase() !== 'lrrm'){
            return;
        }

        setDevice(device);
        setFound(true);
        BLE.stopScan();
    };


    const startScan = async () => {
        if(!(await BLE.isPoweredOn())){
            setBlueError(true);
            Alert.alert('Error', 'Your Bluetooth is turned off. Turn it on and try again!');
            return;
        }
        setBlueError(false);
        await BLE.scanForDevice(addDevice);
    };


    const listenToDevice = async () => {
        try {
            await BLE.connectAndMonitorDevice(device, Timer.displayTimer);
            setIsConnected(true);
        } catch (e) {
            Alert.alert('Error', 'There was an error connecting to your bluetooth device, please try again!')
        }
    };

    useEffect(() => {startScan()}, []);

    return(
        <View style={{flex: 1, alignItems: 'center', justifyContent: 'center'}}>
            <View style={{paddingHorizontal: 30}}>
                {
                    found ? <View>
                            <Text style={{textAlign: 'center', marginBottom: 15}}>{isConnected ? 'Your phone is connected and will react to pushes of the alarm button' : 'We have found your device, would you like to connect to it?'}</Text>
                            {!isConnected && <Button onPress={listenToDevice} title="Yes, Connect Me!"/>}
                        </View> :
                        !blueError && <View>
                            <Text>We are looking for devices</Text>
                            <ActivityIndicator size="large"/>
                        </View>
                }
                {
                    blueError &&
                    <Button title="Retry" onPress={startScan}/>
                }
            </View>
        </View>
    )
}
