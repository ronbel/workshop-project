import PushNotification from 'react-native-push-notification';


export const notify = () => {

    PushNotification.localNotification({
        title: 'You have pressed the emergency button!',
        message: 'Click here to react if it\'s a false alarm!',
        autoCancel: false,
        playSound: true,
        importance: 'max',
        visibility: 'public'
    })
}
