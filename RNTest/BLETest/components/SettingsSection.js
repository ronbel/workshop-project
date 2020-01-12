import React from 'react';
import {View, Text, StyleSheet} from 'react-native';


export default function SettingsSection({title, children}) {


    return (
        <View style={styles.container}>
            <Text style={styles.header}>{title}</Text>
            <View style={{flexDirection: 'row', justifyContent: 'space-between', alignItems: 'center'}}>
                {children}
            </View>
        </View>
    );
}

const styles = StyleSheet.create({
    header: {
        fontSize: 20,
        marginBottom: 10,
    },
    container: {
        paddingHorizontal: 15,
        borderBottomWidth: 1,
        paddingBottom: 20,
        borderColor: 'grey',
        width: '100%',
        marginBottom: 10,
    },
});
