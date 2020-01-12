import React from 'react';
import {Modal, ActivityIndicator, View} from 'react-native';


export default function LoadingModal({visible}) {

    return(
        <Modal animationType="fade" transparent visible={visible} onRequestClose={() => {}} presentationStyle="overFullScreen">
            <View style={{flex: 1, alignItems: 'center', justifyContent: 'center', backgroundColor: 'rgba(0,0,0,0.3)'}}>
                <ActivityIndicator size="large"/>
            </View>
        </Modal>
    )

}
