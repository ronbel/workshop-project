import Geolocation from 'react-native-geolocation-service';


export const getLocationUrl = () => {
    return new Promise((resolve, reject) => {
        Geolocation.getCurrentPosition(
            (position) => {
                const {latitude, longitude} = position.coords;
                resolve(`http://maps.google.com/maps?q=${latitude},${longitude}`);
            },
            (error) => {
                reject(error);
            },
            { enableHighAccuracy: true, timeout: 15000, maximumAge: 10000 }
        );
    })
}
