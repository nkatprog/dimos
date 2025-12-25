/**
 * Robot Stream Screen - Displays video and costmap streams from robot
 * 
 * Copyright 2025 Dimensional Inc.
 */

import React, {useState, useEffect, useRef} from 'react';
import {
  View,
  Text,
  Image,
  TouchableOpacity,
  TextInput,
  ActivityIndicator,
  Alert,
  ScrollView,
  Dimensions,
} from 'react-native';
import {SafeAreaView} from 'react-native-safe-area-context';
import Joystick from '../components/Joystick';
import {BridgeSettings} from './SettingsScreen';

interface RobotStreamScreenProps {
  onBack: () => void;
  bridgeSettings?: BridgeSettings;
}

const RobotStreamScreen: React.FC<RobotStreamScreenProps> = ({onBack, bridgeSettings}) => {
  const [robotIP, setRobotIP] = useState(bridgeSettings?.serverIP || '192.168.1.100');
  const [isConnected, setIsConnected] = useState(false);
  const [isConnecting, setIsConnecting] = useState(false);
  const [availableStreams, setAvailableStreams] = useState<string[]>([]);
  const [error, setError] = useState<string | null>(null);
  const [useMockData, setUseMockData] = useState(false);
  const [currentVideoFrame, setCurrentVideoFrame] = useState(0);
  const [currentLidarFrame, setCurrentLidarFrame] = useState(0);
  const [isLandscape, setIsLandscape] = useState(false);
  
  // Joystick control state
  const [moveX, setMoveX] = useState(0);
  const [moveY, setMoveY] = useState(0);
  const [rotateZ, setRotateZ] = useState(0);
  const controlTimerRef = useRef<NodeJS.Timeout | null>(null);

  const serverPort = bridgeSettings?.serverPort || '5555';
  const serverUrl = `http://${robotIP}:${serverPort}`;

  // Mock data frames from actual unitree_office_walk data
  // Note: React Native requires static require() calls - Metro bundler needs
  // to know all assets at compile time. We use a mapping object for cleaner code.
  const MOCK_FRAMES = {
    video: {
      count: 50,
      getFrame: (i: number) => {
        // This mapping satisfies Metro's static analysis while reducing boilerplate
        const frames: Record<number, any> = {
          0: require('../../assets/mock_data/video/frame_000.jpg'),
          1: require('../../assets/mock_data/video/frame_001.jpg'),
          2: require('../../assets/mock_data/video/frame_002.jpg'),
          3: require('../../assets/mock_data/video/frame_003.jpg'),
          4: require('../../assets/mock_data/video/frame_004.jpg'),
          5: require('../../assets/mock_data/video/frame_005.jpg'),
          6: require('../../assets/mock_data/video/frame_006.jpg'),
          7: require('../../assets/mock_data/video/frame_007.jpg'),
          8: require('../../assets/mock_data/video/frame_008.jpg'),
          9: require('../../assets/mock_data/video/frame_009.jpg'),
          10: require('../../assets/mock_data/video/frame_010.jpg'),
          11: require('../../assets/mock_data/video/frame_011.jpg'),
          12: require('../../assets/mock_data/video/frame_012.jpg'),
          13: require('../../assets/mock_data/video/frame_013.jpg'),
          14: require('../../assets/mock_data/video/frame_014.jpg'),
          15: require('../../assets/mock_data/video/frame_015.jpg'),
          16: require('../../assets/mock_data/video/frame_016.jpg'),
          17: require('../../assets/mock_data/video/frame_017.jpg'),
          18: require('../../assets/mock_data/video/frame_018.jpg'),
          19: require('../../assets/mock_data/video/frame_019.jpg'),
          20: require('../../assets/mock_data/video/frame_020.jpg'),
          21: require('../../assets/mock_data/video/frame_021.jpg'),
          22: require('../../assets/mock_data/video/frame_022.jpg'),
          23: require('../../assets/mock_data/video/frame_023.jpg'),
          24: require('../../assets/mock_data/video/frame_024.jpg'),
          25: require('../../assets/mock_data/video/frame_025.jpg'),
          26: require('../../assets/mock_data/video/frame_026.jpg'),
          27: require('../../assets/mock_data/video/frame_027.jpg'),
          28: require('../../assets/mock_data/video/frame_028.jpg'),
          29: require('../../assets/mock_data/video/frame_029.jpg'),
          30: require('../../assets/mock_data/video/frame_030.jpg'),
          31: require('../../assets/mock_data/video/frame_031.jpg'),
          32: require('../../assets/mock_data/video/frame_032.jpg'),
          33: require('../../assets/mock_data/video/frame_033.jpg'),
          34: require('../../assets/mock_data/video/frame_034.jpg'),
          35: require('../../assets/mock_data/video/frame_035.jpg'),
          36: require('../../assets/mock_data/video/frame_036.jpg'),
          37: require('../../assets/mock_data/video/frame_037.jpg'),
          38: require('../../assets/mock_data/video/frame_038.jpg'),
          39: require('../../assets/mock_data/video/frame_039.jpg'),
          40: require('../../assets/mock_data/video/frame_040.jpg'),
          41: require('../../assets/mock_data/video/frame_041.jpg'),
          42: require('../../assets/mock_data/video/frame_042.jpg'),
          43: require('../../assets/mock_data/video/frame_043.jpg'),
          44: require('../../assets/mock_data/video/frame_044.jpg'),
          45: require('../../assets/mock_data/video/frame_045.jpg'),
          46: require('../../assets/mock_data/video/frame_046.jpg'),
          47: require('../../assets/mock_data/video/frame_047.jpg'),
          48: require('../../assets/mock_data/video/frame_048.jpg'),
          49: require('../../assets/mock_data/video/frame_049.jpg'),
        };
        return frames[i];
      }
    },
    lidar: {
      count: 50,
      getFrame: (i: number) => {
        const frames: Record<number, any> = {
          0: require('../../assets/mock_data/lidar/lidar_000.jpg'),
          1: require('../../assets/mock_data/lidar/lidar_001.jpg'),
          2: require('../../assets/mock_data/lidar/lidar_002.jpg'),
          3: require('../../assets/mock_data/lidar/lidar_003.jpg'),
          4: require('../../assets/mock_data/lidar/lidar_004.jpg'),
          5: require('../../assets/mock_data/lidar/lidar_005.jpg'),
          6: require('../../assets/mock_data/lidar/lidar_006.jpg'),
          7: require('../../assets/mock_data/lidar/lidar_007.jpg'),
          8: require('../../assets/mock_data/lidar/lidar_008.jpg'),
          9: require('../../assets/mock_data/lidar/lidar_009.jpg'),
          10: require('../../assets/mock_data/lidar/lidar_010.jpg'),
          11: require('../../assets/mock_data/lidar/lidar_011.jpg'),
          12: require('../../assets/mock_data/lidar/lidar_012.jpg'),
          13: require('../../assets/mock_data/lidar/lidar_013.jpg'),
          14: require('../../assets/mock_data/lidar/lidar_014.jpg'),
          15: require('../../assets/mock_data/lidar/lidar_015.jpg'),
          16: require('../../assets/mock_data/lidar/lidar_016.jpg'),
          17: require('../../assets/mock_data/lidar/lidar_017.jpg'),
          18: require('../../assets/mock_data/lidar/lidar_018.jpg'),
          19: require('../../assets/mock_data/lidar/lidar_019.jpg'),
          20: require('../../assets/mock_data/lidar/lidar_020.jpg'),
          21: require('../../assets/mock_data/lidar/lidar_021.jpg'),
          22: require('../../assets/mock_data/lidar/lidar_022.jpg'),
          23: require('../../assets/mock_data/lidar/lidar_023.jpg'),
          24: require('../../assets/mock_data/lidar/lidar_024.jpg'),
          25: require('../../assets/mock_data/lidar/lidar_025.jpg'),
          26: require('../../assets/mock_data/lidar/lidar_026.jpg'),
          27: require('../../assets/mock_data/lidar/lidar_027.jpg'),
          28: require('../../assets/mock_data/lidar/lidar_028.jpg'),
          29: require('../../assets/mock_data/lidar/lidar_029.jpg'),
          30: require('../../assets/mock_data/lidar/lidar_030.jpg'),
          31: require('../../assets/mock_data/lidar/lidar_031.jpg'),
          32: require('../../assets/mock_data/lidar/lidar_032.jpg'),
          33: require('../../assets/mock_data/lidar/lidar_033.jpg'),
          34: require('../../assets/mock_data/lidar/lidar_034.jpg'),
          35: require('../../assets/mock_data/lidar/lidar_035.jpg'),
          36: require('../../assets/mock_data/lidar/lidar_036.jpg'),
          37: require('../../assets/mock_data/lidar/lidar_037.jpg'),
          38: require('../../assets/mock_data/lidar/lidar_038.jpg'),
          39: require('../../assets/mock_data/lidar/lidar_039.jpg'),
          40: require('../../assets/mock_data/lidar/lidar_040.jpg'),
          41: require('../../assets/mock_data/lidar/lidar_041.jpg'),
          42: require('../../assets/mock_data/lidar/lidar_042.jpg'),
          43: require('../../assets/mock_data/lidar/lidar_043.jpg'),
          44: require('../../assets/mock_data/lidar/lidar_044.jpg'),
          45: require('../../assets/mock_data/lidar/lidar_045.jpg'),
          46: require('../../assets/mock_data/lidar/lidar_046.jpg'),
          47: require('../../assets/mock_data/lidar/lidar_047.jpg'),
          48: require('../../assets/mock_data/lidar/lidar_048.jpg'),
          49: require('../../assets/mock_data/lidar/lidar_049.jpg'),
        };
        return frames[i];
      }
    }
  };

  // Fetch available streams from server
  const fetchStreams = async () => {
    try {
      const response = await fetch(`${serverUrl}/streams`, {
        method: 'GET',
        headers: {
          Accept: 'application/json',
        },
      });

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      const data = await response.json();
      return data.streams || [];
    } catch (err) {
      throw new Error(
        `Failed to connect: ${err instanceof Error ? err.message : 'Unknown error'}`,
      );
    }
  };

  // Connect to robot (or use mock data)
  const handleConnect = async () => {
    setIsConnecting(true);
    setError(null);

    try {
      const streams = await fetchStreams();

      if (streams.length === 0) {
        throw new Error('No streams available from robot');
      }

      setAvailableStreams(streams);
      setIsConnected(true);
      setUseMockData(false);
      console.log('Connected to robot:', streams);
    } catch (err) {
      const errorMsg =
        err instanceof Error ? err.message : 'Failed to connect to robot';
      setError(errorMsg);
      Alert.alert('Connection Error', errorMsg);
      setIsConnected(false);
    } finally {
      setIsConnecting(false);
    }
  };

  // Use mock data without connecting to server
  const handleUseMockData = () => {
    setUseMockData(true);
    setIsConnected(true);
    setAvailableStreams(['mock_video', 'mock_costmap']);
    setError(null);
  };

  // Disconnect from robot
  const handleDisconnect = () => {
    setIsConnected(false);
    setAvailableStreams([]);
    setUseMockData(false);
    setError(null);
    
    // Stop any active control commands
    if (controlTimerRef.current) {
      clearInterval(controlTimerRef.current);
      controlTimerRef.current = null;
    }
  };

  // Cycle through mock video and lidar frames
  useEffect(() => {
    if (useMockData && isConnected) {
      const interval = setInterval(() => {
        setCurrentVideoFrame(prev => (prev + 1) % MOCK_FRAMES.video.count);
        setCurrentLidarFrame(prev => (prev + 1) % MOCK_FRAMES.lidar.count);
      }, 100); // Change frame every 100ms (10 fps)

      return () => clearInterval(interval);
    }
  }, [useMockData, isConnected]);

  // Send control command to robot (with throttling to avoid spam)
  const lastCommandTime = useRef(0);
  const COMMAND_THROTTLE_MS = 50; // Min 50ms between commands (20 Hz max)

  const sendControlCommand = async (vx: number, vy: number, vz: number) => {
    // Throttle commands to avoid overwhelming the server
    const now = Date.now();
    if (now - lastCommandTime.current < COMMAND_THROTTLE_MS) {
      return;
    }
    lastCommandTime.current = now;

    // Skip sending commands in mock mode
    if (useMockData) {
      return;
    }

    try {
      await fetch(`${serverUrl}/control`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          linear_x: vx,
          linear_y: vy,
          angular_z: vz,
        }),
      });
    } catch (err) {
      console.error('Failed to send control command:', err);
    }
  };

  // Handle movement joystick
  const handleMoveJoystick = (x: number, y: number) => {
    setMoveX(x);
    setMoveY(y);
    
    // Send command immediately
    sendControlCommand(y, -x, rotateZ);
  };

  // Handle rotation joystick
  const handleRotateJoystick = (x: number, y: number) => {
    // Use x-axis for rotation
    setRotateZ(x);
    
    // Send command immediately
    sendControlCommand(moveY, -moveX, x);
  };

  // Handle joystick release
  const handleJoystickRelease = () => {
    sendControlCommand(0, 0, 0);
  };

  // Detect orientation changes
  useEffect(() => {
    const updateOrientation = () => {
      const {width, height} = Dimensions.get('window');
      setIsLandscape(width > height);
    };

    updateOrientation();
    const subscription = Dimensions.addEventListener('change', updateOrientation);
    
    return () => subscription?.remove();
  }, []);

  // Auto-connect on mount if IP is set
  useEffect(() => {
    if (robotIP && !isConnected) {
      // Optional: auto-connect on load
      // handleConnect();
    }
  }, []);

  return (
    <SafeAreaView className={`flex-1 ${isConnected ? 'bg-black' : 'bg-dimos-blue'}`}>
      {!isConnected && (
        <>
          {/* Back Button - Top left for connection screen */}
          <View className="px-6 pt-4 pb-2">
            <TouchableOpacity
              onPress={onBack}
              className="w-10 h-10 rounded-full bg-dimos-yellow/20 items-center justify-center self-start">
              <Text className="text-dimos-yellow text-lg">←</Text>
            </TouchableOpacity>
          </View>

          {/* Connection Panel */}
        <View className="flex-1 p-6 justify-center">
          <Text className="text-dimos-yellow text-base font-bold mb-2">
            Robot IP Address:
          </Text>
          <TextInput
            className="bg-white rounded-lg p-3 text-base mb-4"
            value={robotIP}
            onChangeText={setRobotIP}
            placeholder="192.168.1.100"
            keyboardType="decimal-pad"
            autoCapitalize="none"
            editable={!isConnecting}
          />

          <TouchableOpacity
            className={`bg-dimos-yellow rounded-lg p-4 items-center ${isConnecting ? 'opacity-60' : ''}`}
            onPress={handleConnect}
            disabled={isConnecting}>
            {isConnecting ? (
              <ActivityIndicator color="#0016B1" />
            ) : (
              <Text className="text-dimos-blue text-lg font-bold">Connect to Robot</Text>
            )}
          </TouchableOpacity>

          {error && (
            <View className="bg-red-500/20 rounded-lg p-3 mt-4">
              <Text className="text-red-400 text-sm">{error}</Text>
            </View>
          )}

          <View className="mt-4">
            <View className="flex-row items-center my-3">
              <View className="flex-1 h-0.5 bg-dimos-yellow/30" />
              <Text className="text-dimos-yellow/50 text-xs mx-3">OR</Text>
              <View className="flex-1 h-0.5 bg-dimos-yellow/30" />
            </View>

            <TouchableOpacity
              className="bg-dimos-yellow/20 border-2 border-dimos-yellow rounded-lg p-4 items-center"
              onPress={handleUseMockData}>
              <Text className="text-dimos-yellow text-lg font-bold">🎬 Use Mock Data</Text>
              <Text className="text-dimos-yellow/70 text-xs mt-1">Test without robot</Text>
            </TouchableOpacity>
          </View>

          <View className="mt-6 p-4 bg-dimos-yellow/10 rounded-lg">
            <Text className="text-dimos-yellow text-xs mb-1">
              Make sure your phone and robot are on the same WiFi network.
            </Text>
            <Text className="text-dimos-yellow text-xs">
              Server runs on port {serverPort}
            </Text>
          </View>
        </View>
        </>
      )}

      {/* Full-Screen Black View - Only when connected */}
      {isConnected && (
        <View className="flex-1 bg-black">
          {/* Back Button - Top left for stream view */}
          <View className="absolute top-4 left-4 z-10">
            <TouchableOpacity
              onPress={onBack}
              className="w-10 h-10 rounded-full bg-white/10 items-center justify-center">
              <Text className="text-white text-lg">←</Text>
            </TouchableOpacity>
          </View>

          {/* Camera and Lidar Streams */}
          <View className={isLandscape ? "flex-1 pb-28" : "flex-1 pb-32"}>
            <View
              className="flex-1"
              style={{
                flexDirection: isLandscape ? 'row' : 'column',
              }}>
              {/* Video Stream */}
              {(availableStreams.includes('unitree_video') || availableStreams.includes('mock_video')) && (
                <View className="flex-1">
                  <Image
                    source={useMockData
                      ? MOCK_FRAMES.video.getFrame(currentVideoFrame)
                      : {uri: `${serverUrl}/video_feed/unitree_video`}
                    }
                    className="w-full h-full bg-black"
                    resizeMode="contain"
                  />
                </View>
              )}

              {/* Lidar/Costmap Stream */}
              {(availableStreams.includes('unitree_costmap') || availableStreams.includes('mock_costmap')) && (
                <View className="flex-1">
                  <Image
                    source={useMockData
                      ? MOCK_FRAMES.lidar.getFrame(currentLidarFrame)
                      : {uri: `${serverUrl}/video_feed/unitree_costmap`}
                    }
                    className="w-full h-full bg-black"
                    resizeMode="contain"
                  />
                </View>
              )}
            </View>
          </View>

          {/* Joystick Controls - Always at Bottom */}
          <View className="absolute bottom-0 left-0 right-0 bg-black/80" style={{
            paddingBottom: 20,
            paddingTop: 16,
          }}>
            <View
              className="flex-row justify-around items-center"
              style={{
                paddingHorizontal: isLandscape ? 80 : 40,
              }}>
              {/* Left Joystick - Movement */}
              <Joystick
                onMove={handleMoveJoystick}
                onRelease={handleJoystickRelease}
                label=""
                size={isLandscape ? 80 : 100}
                thumbSize={isLandscape ? 32 : 40}
                color="#FFFFFF"
              />

              {/* Right Joystick - Rotation */}
              <Joystick
                onMove={handleRotateJoystick}
                onRelease={handleJoystickRelease}
                label=""
                size={isLandscape ? 80 : 100}
                thumbSize={isLandscape ? 32 : 40}
                color="#FFFFFF"
              />
            </View>
          </View>
        </View>
      )}
    </SafeAreaView>
  );
};

export default RobotStreamScreen;

