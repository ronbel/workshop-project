import React, {useState} from 'react';
import {Modal,Animated, View, Text, StyleSheet, Button, Alert} from 'react-native';
import CountDown from 'react-native-countdown-component';


export default function TimerModal({visible, onFalseAlarmPress}) {


    return(
        <Modal presentationStyle="overFullScreen" animationType="fade" visible={visible} onRequestClose={() => {}}>
            <View style={styles.container}>
                <View style={styles.box}>
                    <Text style={{fontSize: 20}}>Is Everything OK?</Text>
                    <Text style={{textAlign: 'center'}}>We noticed you pressed the alarm button. Just to make sure, we're giving you a chance to inform us everything is OK and avoid false alrams</Text>

                    <CountDown timeLabels={{s: ''}} timeToShow={['S']} until={11} size={20} onFinish={() => Alert.alert('Ah shit!', 'Nigga in trouble, call the police')}/>

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
