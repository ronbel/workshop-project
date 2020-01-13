import React , {useState, useEffect} from 'react';
import {Modal, View, Text, StyleSheet, Button, Alert} from 'react-native';
import CountDown from 'react-native-countdown-component';
import Storage from '../services/storage-service';
import SMSService from '../services/SMS-service';


export const ModalContext = React.createContext(null);

export default function TimerModal({visible, onFalseAlarmPress}) {

    const [secondsToWait, setSecondsToWait] = useState(0);

    const onModalAppear = async () => {
        const settings = await Storage.get_settings();
        setSecondsToWait(settings.secondsToWait)
    };

    const onTimerFinished = async () => {
        await SMSService.getAndSend();
    };

    useEffect(() => {
        if(visible){
            onModalAppear();
        }
    }, [visible]);



    return(
        <Modal presentationStyle="overFullScreen" animationType="fade" visible={visible} onRequestClose={() => {}}>
            <View style={styles.container}>
                <View style={styles.box}>
                    <Text style={{fontSize: 20}}>Is Everything OK?</Text>
                    <Text style={{textAlign: 'center'}}>We noticed you pressed the alarm button. Just to make sure, we're giving you a chance to inform us everything is OK and avoid false alarms</Text>

                    <CountDown timeLabels={{s: ''}} timeToShow={['S']} until={secondsToWait} size={20} onFinish={onTimerFinished}/>

                    <Button title="It's OK, false alarm!" onPress={onFalseAlarmPress}/>

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
        paddingVertical: 40,
        paddingHorizontal: 5
    }
})
