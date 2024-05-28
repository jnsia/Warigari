import React from 'react';
import {createStackNavigator} from '@react-navigation/stack';
import {Alert, BackHandler, Image, TouchableOpacity} from 'react-native';

const HomeStack = createStackNavigator();
import Home from './views/Home';

export const HomeStackNavigator = () => {
    return (
      <HomeStack.Navigator initialRouteName="Home">
        <HomeStack.Screen
          name="Home"
          component={Home}
          options={{
            title: 'WARIGARI',
            headerBackTitleVisible: false,
            headerStyle: {
              backgroundColor: 'black',
              height: 75,
            },
            headerTintColor: 'white',
            headerTitleStyle: {
              fontWeight: 'bold',
            },
          }}
        />
      </HomeStack.Navigator>
    );
  };
  