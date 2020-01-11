import React, {useState, useEffect} from 'react';
import {Alert, Button, NativeModules, PermissionsAndroid, SafeAreaView, StatusBar, ToastAndroid, View, Text, ActivityIndicator} from 'react-native';
import BLE from './services/ble-service';
import TimerModal from './components/TimerModal';







const App: () => React$Node = () => {
    const [device, setDevice] = useState(null);
    const [found, setFound] = useState(false);
    const [blueError, setBlueError]  = useState(false);
    const [showTimer, setShowTimer] = useState(false);


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

    const toggleTimer = () => setShowTimer(!showTimer);

    const listenToDevice = async () => await BLE.connectAndMonitorDevice(device, toggleTimer);

    useEffect(() => {startScan()}, []);

    return (
        <>
            <StatusBar barStyle="dark-content"/>
            <TimerModal onFalseAlarmPress={toggleTimer} visible={showTimer}/>
            <SafeAreaView
                style={{flex: 1, alignItems: 'center', justifyContent: 'center'}}>
                    <View style={{paddingHorizontal: 30}}>
                        {
                           found ? <View>
                                <Text>We have found your device, would you like to connect to it?</Text>
                                <Button onPress={listenToDevice} title="Yalla!"/>
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
            </SafeAreaView>
        </>
    );
};

export default App;
