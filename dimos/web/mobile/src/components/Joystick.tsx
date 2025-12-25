/**
 * Joystick Component - Touch-based joystick control
 * 
 * Copyright 2025 Dimensional Inc.
 */

import React, {useRef, useState} from 'react';
import {
  View,
  PanResponder,
  StyleSheet,
  Text,
} from 'react-native';

interface JoystickProps {
  onMove: (x: number, y: number) => void;
  onRelease?: () => void;
  size?: number;
  thumbSize?: number;
  label?: string;
  color?: string;
}

const Joystick: React.FC<JoystickProps> = ({
  onMove,
  onRelease,
  size = 120,
  thumbSize = 50,
  label = '',
  color = '#FFE500',
}) => {
  const [position, setPosition] = useState({x: 0, y: 0});
  const maxDistance = (size - thumbSize) / 2;

  const panResponder = useRef(
    PanResponder.create({
      onStartShouldSetPanResponder: () => true,
      onMoveShouldSetPanResponder: () => true,
      onPanResponderGrant: () => {
        setPosition({x: 0, y: 0});
      },
      onPanResponderMove: (_, gestureState) => {
        const {dx, dy} = gestureState;
        
        // Calculate distance from center
        const distance = Math.sqrt(dx * dx + dy * dy);
        
        // Limit to max distance
        let newX = dx;
        let newY = dy;
        
        if (distance > maxDistance) {
          const angle = Math.atan2(dy, dx);
          newX = Math.cos(angle) * maxDistance;
          newY = Math.sin(angle) * maxDistance;
        }
        
        setPosition({x: newX, y: newY});
        
        // Normalize to -1 to 1 range
        const normalizedX = newX / maxDistance;
        const normalizedY = -newY / maxDistance; // Invert Y axis (up is positive)
        
        onMove(normalizedX, normalizedY);
      },
      onPanResponderRelease: () => {
        setPosition({x: 0, y: 0});
        onMove(0, 0);
        if (onRelease) {
          onRelease();
        }
      },
    }),
  ).current;

  return (
    <View style={styles.container}>
      {label && <Text style={[styles.label, {color}]}>{label}</Text>}
      <View
        style={[
          styles.joystickBase,
          {
            width: size,
            height: size,
            borderRadius: size / 2,
            borderColor: color,
          },
        ]}>
        <View
          {...panResponder.panHandlers}
          style={[
            styles.joystickThumb,
            {
              width: thumbSize,
              height: thumbSize,
              borderRadius: thumbSize / 2,
              backgroundColor: color,
              transform: [
                {translateX: position.x},
                {translateY: position.y},
              ],
            },
          ]}
        />
      </View>
    </View>
  );
};

const styles = StyleSheet.create({
  container: {
    alignItems: 'center',
    justifyContent: 'center',
  },
  label: {
    fontSize: 12,
    fontWeight: 'bold',
    marginBottom: 8,
  },
  joystickBase: {
    borderWidth: 3,
    justifyContent: 'center',
    alignItems: 'center',
    backgroundColor: 'rgba(0, 0, 0, 0.3)',
  },
  joystickThumb: {
    shadowColor: '#000',
    shadowOffset: {
      width: 0,
      height: 2,
    },
    shadowOpacity: 0.5,
    shadowRadius: 3.84,
    elevation: 5,
  },
});

export default Joystick;


