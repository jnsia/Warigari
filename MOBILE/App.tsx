/**
 * Sample React Native App
 * https://github.com/facebook/react-native
 *
 * @format
 */

import {NavigationContainer} from '@react-navigation/native';
import React from 'react';
import {StyleSheet} from 'react-native';
import {HomeStackNavigator} from './Route';
import {createStackNavigator} from '@react-navigation/stack';
import ROSLIB from 'roslib';

const HomeStack = createStackNavigator();

function App(): React.JSX.Element {
  return (
    <NavigationContainer>
      <HomeStack.Navigator screenOptions={{headerShown: false}}>
        <HomeStack.Screen name="HomeStack" component={HomeStackNavigator} />
      </HomeStack.Navigator>
    </NavigationContainer>
  );
}

const styles = StyleSheet.create({});

export default App;
