import React , {useState, useEffect} from 'react';
import {Modal, View, Text, StyleSheet, Button, Image} from 'react-native';
import CountDown from 'react-native-countdown-component';
import Storage from '../services/storage-service';
import SMSService from '../services/SMS-service';


export const ModalContext = React.createContext(null);

export default function TimerModal({visible, onFalseAlarmPress}) {

    const [secondsToWait, setSecondsToWait] = useState(0);
    const [emergencyDetected, setEmergencyDetected] = useState(false);

    const onModalAppear = async () => {
        setEmergencyDetected(false);
        const settings = await Storage.get_settings();
        setSecondsToWait(settings.secondsToWait)
    };

    const onTimerFinished = async () => {
        setEmergencyDetected(true);
        await SMSService.getAndSend();
    };

    useEffect(() => {
        if(visible){
            onModalAppear();
        }
    }, [visible]);



    return(
        <Modal presentationStyle="overFullScreen" transparent animationType="fade" visible={visible} onRequestClose={() => {}}>
            <View style={styles.container}>
                <View style={styles.box}>
                    <Text style={{fontSize: 20}}>{emergencyDetected ? 'Emergency Alert!' : 'Is Everything OK?'}</Text>
                    <Text style={{textAlign: 'center'}}>{emergencyDetected ? 'We have detected a real emergency and sent a message to your selected contacts. Help is on the way!' : 'We noticed you pressed the alarm button. Just to make sure, we\'re giving you a chance to inform us everything is OK and avoid false alarms'}</Text>

                    {!emergencyDetected && <CountDown timeLabels={{s: ''}} timeToShow={['S']} until={secondsToWait} size={20} onFinish={onTimerFinished}/>}
                    {emergencyDetected && <Image source={require('../assets/images/siren.png')} style={{height: 120, width: 120, marginVertical: 20}} />}
                    <Button title={emergencyDetected ? 'Got it' : "It's OK, false alarm!"} onPress={onFalseAlarmPress}/>

                </View>
            </View>
        </Modal>
    )
}

const styles = StyleSheet.create({
    container: {
        flex: 1,
        justifyContent: 'center',
        alignItems: 'center',
        backgroundColor: 'rgba(0,0,0,0.3)'
    },
    box: {
        backgroundColor: 'white',
        width: '80%',
        height: '50%',
        borderRadius: 10,
        alignItems: 'center',
        justifyContent: 'space-between',
        paddingVertical: 30,
        paddingHorizontal: 10
    }
})
