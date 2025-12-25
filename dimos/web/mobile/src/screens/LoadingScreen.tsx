import React from 'react';
import {StatusBar, View} from 'react-native';
import {SafeAreaView} from 'react-native-safe-area-context';
import FigletText from '../utils/FigletText';

const LoadingScreen: React.FC = () => {
  return (
    <SafeAreaView className="flex-1 bg-dimos-blue justify-center">
      <StatusBar barStyle="light-content" />
      <View className="px-6 items-center">
        <FigletText text="DIMENSIONAL" color="#FFF200" fontSize={4} />
      </View>
    </SafeAreaView>
  );
};

export default LoadingScreen;
