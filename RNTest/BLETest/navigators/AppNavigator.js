import {createBottomTabNavigator} from 'react-navigation-tabs';
import {createAppContainer} from 'react-navigation';
import Connect from '../screens/Connect';
import Contacts from '../screens/Contacts';
import Settings from '../screens/Settings';
import React from 'react';
import {Image} from 'react-native';

const ConnectIcon = require('../assets/images/bluetooth.png');
const ContactsIcon = require('../assets/images/contact.png');
const SettingsIcon = require('../assets/images/wheel.png');

const AppNavigator = createBottomTabNavigator({
        Connect: {
            screen: Connect,
            tabBarLabel: 'Connect',
            navigationOptions: {
                tabBarIcon: () => <Image source={ConnectIcon} style={{height: 25, width: 25}}/>
            }
        },
        Contacts: {
            screen: Contacts,
            tabBarLabel: 'Contacts',
            navigationOptions: {
                tabBarIcon: () => <Image source={ContactsIcon} style={{height: 25, width: 25}}/>
            }
        },
        Settings: {
            screen: Settings,
            tabBarLabel: 'Settings',
            navigationOptions: {
                tabBarIcon: () => <Image source={SettingsIcon} style={{height: 25, width: 25}}/>
            }
        }
    }
, {
        initialRouteName: 'Connect'
    })


export default createAppContainer(AppNavigator);
