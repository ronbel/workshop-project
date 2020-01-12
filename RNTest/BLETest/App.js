import React, {useState} from 'react';
import {StatusBar, View} from 'react-native';
import TimerModal, {ModalContext} from './components/TimerModal';
import AppNavigator from './navigators/AppNavigator';


const App: () => React$Node = () => {
    const [showTimer, setShowTimer] = useState(false);
    const displayTimer = () => setShowTimer(true);
    const hideTimer = () => setShowTimer(false);


    return (
        <View style={{flex: 1}}>
            <StatusBar hidden barStyle="dark-content"/>
            <TimerModal onFalseAlarmPress={hideTimer} visible={showTimer}/>
            <ModalContext.Provider value={{displayTimer, hideTimer}}>
                <AppNavigator/>
            </ModalContext.Provider>
        </View>
    );
};

export default App;
