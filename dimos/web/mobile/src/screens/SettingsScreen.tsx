/**
 * Settings Screen - Configure LCM Bridge Server
 * 
 * Allows users to configure LCM channel names and server settings
 * directly from the mobile app UI.
 */

import React, {useState, useEffect} from 'react';
import {
  View,
  Text,
  TextInput,
  TouchableOpacity,
  ScrollView,
  Alert,
  ActivityIndicator,
} from 'react-native';
import {SafeAreaView} from 'react-native-safe-area-context';

interface SettingsScreenProps {
  onBack: () => void;
  onSave: (settings: BridgeSettings) => void;
}

export interface BridgeSettings {
  serverIP: string;
  serverPort: string;
  videoChannel: string;
  lidarChannel: string;
  controlChannel: string;
  lcmUrl: string;
}

const SettingsScreen: React.FC<SettingsScreenProps> = ({onBack, onSave}) => {
  const [settings, setSettings] = useState<BridgeSettings>({
    serverIP: 'localhost',
    serverPort: '5555',
    videoChannel: 'UNITREE_VIDEO',
    lidarChannel: 'UNITREE_LIDAR',
    controlChannel: 'UNITREE_CONTROL',
    lcmUrl: 'udpm://239.255.76.67:7667?ttl=1',
  });

  const [isSaving, setIsSaving] = useState(false);
  const [isDetecting, setIsDetecting] = useState(false);
  const [detectedChannels, setDetectedChannels] = useState<string[]>([]);

  // Load saved settings from bridge server
  useEffect(() => {
    loadSettings();
  }, []);

  const loadSettings = async () => {
    try {
      const response = await fetch(`http://${settings.serverIP}:${settings.serverPort}/config`);
      if (response.ok) {
        const config = await response.json();
        setSettings(prev => ({
          ...prev,
          ...config,
        }));
      }
    } catch (err) {
      // Server not running or not reachable, use defaults
      console.log('Could not load settings from server, using defaults');
    }
  };

  const handleSave = async () => {
    setIsSaving(true);
    try {
      // Send configuration to bridge server
      const response = await fetch(`http://${settings.serverIP}:${settings.serverPort}/config`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(settings),
      });

      if (response.ok) {
        Alert.alert(
          'Settings Saved',
          'Bridge server configuration updated. Please restart the server for changes to take effect.',
          [
            {
              text: 'Restart Server',
              onPress: handleRestartServer,
            },
            {
              text: 'OK',
              onPress: () => onSave(settings),
            },
          ]
        );
      } else {
        throw new Error('Failed to save settings');
      }
    } catch (err) {
      Alert.alert(
        'Error',
        'Could not save settings to bridge server. Make sure the server is running.',
        [{text: 'OK'}]
      );
    } finally {
      setIsSaving(false);
    }
  };

  const handleRestartServer = async () => {
    try {
      await fetch(`http://${settings.serverIP}:${settings.serverPort}/restart`, {
        method: 'POST',
      });
      Alert.alert('Server Restarting', 'The bridge server is restarting with new settings...');
      onSave(settings);
    } catch (err) {
      Alert.alert('Error', 'Could not restart server. Please restart manually.');
    }
  };

  const handleDetectChannels = async () => {
    setIsDetecting(true);
    setDetectedChannels([]);
    
    try {
      const response = await fetch(`http://${settings.serverIP}:${settings.serverPort}/detect-channels`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          lcmUrl: settings.lcmUrl,
          timeout: 5000, // 5 second scan
        }),
      });

      if (response.ok) {
        const data = await response.json();
        setDetectedChannels(data.channels || []);
        
        if (data.channels && data.channels.length > 0) {
          Alert.alert(
            'Channels Detected',
            `Found ${data.channels.length} active LCM channels. Tap a channel name below to use it.`,
            [{text: 'OK'}]
          );
        } else {
          Alert.alert(
            'No Channels Found',
            'No active LCM channels detected. Make sure your robot is running and publishing data.',
            [{text: 'OK'}]
          );
        }
      } else {
        throw new Error('Detection failed');
      }
    } catch (err) {
      Alert.alert(
        'Detection Error',
        'Could not detect LCM channels. Make sure the bridge server is running.',
        [{text: 'OK'}]
      );
    } finally {
      setIsDetecting(false);
    }
  };

  const applyDetectedChannel = (channel: string, type: 'video' | 'lidar' | 'control') => {
    setSettings(prev => ({
      ...prev,
      [`${type}Channel`]: channel,
    }));
    Alert.alert('Channel Applied', `${type.toUpperCase()} channel set to: ${channel}`);
  };

  return (
    <SafeAreaView className="flex-1 bg-dimos-blue">
      {/* Back Button - Top left */}
      <View className="px-6 pt-4 pb-2">
        <TouchableOpacity
          onPress={onBack}
          className="w-10 h-10 rounded-full bg-dimos-yellow/20 items-center justify-center self-start">
          <Text className="text-dimos-yellow text-lg">←</Text>
        </TouchableOpacity>
      </View>

      <ScrollView className="flex-1 px-6 pb-6">
        <View className="mb-6">
          <Text className="text-dimos-yellow text-2xl font-bold mb-2">
            Bridge Server Settings
          </Text>
          <Text className="text-dimos-yellow/70 text-sm">
            Configure LCM channels and server connection
          </Text>
        </View>

        {/* Server Connection */}
        <View className="mb-6">
          <Text className="text-dimos-yellow text-lg font-bold mb-3">
            Server Connection
          </Text>
          
          <Text className="text-dimos-yellow text-sm mb-1">Server IP Address:</Text>
          <TextInput
            className="bg-white rounded-lg p-3 text-base mb-3"
            value={settings.serverIP}
            onChangeText={(text) => setSettings(prev => ({...prev, serverIP: text}))}
            placeholder="localhost or 192.168.1.100"
            keyboardType="default"
            autoCapitalize="none"
          />

          <Text className="text-dimos-yellow text-sm mb-1">Server Port:</Text>
          <TextInput
            className="bg-white rounded-lg p-3 text-base mb-3"
            value={settings.serverPort}
            onChangeText={(text) => setSettings(prev => ({...prev, serverPort: text}))}
            placeholder="5555"
            keyboardType="number-pad"
          />
        </View>

        {/* LCM Configuration */}
        <View className="mb-6">
          <Text className="text-dimos-yellow text-lg font-bold mb-3">
            LCM Configuration
          </Text>

          <Text className="text-dimos-yellow text-sm mb-1">LCM URL:</Text>
          <TextInput
            className="bg-white rounded-lg p-3 text-base mb-3"
            value={settings.lcmUrl}
            onChangeText={(text) => setSettings(prev => ({...prev, lcmUrl: text}))}
            placeholder="udpm://239.255.76.67:7667?ttl=1"
            autoCapitalize="none"
          />
        </View>

        {/* Channel Detection */}
        <View className="mb-6">
          <TouchableOpacity
            className={`bg-dimos-yellow/20 border-2 border-dimos-yellow rounded-lg p-4 items-center ${isDetecting ? 'opacity-60' : ''}`}
            onPress={handleDetectChannels}
            disabled={isDetecting}>
            {isDetecting ? (
              <ActivityIndicator color="#FCD34D" />
            ) : (
              <>
                <Text className="text-dimos-yellow text-lg font-bold">🔍 Detect LCM Channels</Text>
                <Text className="text-dimos-yellow/70 text-xs mt-1">
                  Scan for active channels (5 seconds)
                </Text>
              </>
            )}
          </TouchableOpacity>

          {detectedChannels.length > 0 && (
            <View className="mt-4 p-4 bg-dimos-yellow/10 rounded-lg">
              <Text className="text-dimos-yellow text-sm font-bold mb-2">
                Detected Channels ({detectedChannels.length}):
              </Text>
              <ScrollView className="max-h-40">
                {detectedChannels.map((channel, index) => (
                  <View key={index} className="flex-row items-center mb-2">
                    <Text className="text-dimos-yellow text-xs flex-1 font-mono">
                      {channel}
                    </Text>
                    <View className="flex-row space-x-2">
                      <TouchableOpacity
                        onPress={() => applyDetectedChannel(channel, 'video')}
                        className="bg-dimos-yellow/20 px-2 py-1 rounded">
                        <Text className="text-dimos-yellow text-xs">Video</Text>
                      </TouchableOpacity>
                      <TouchableOpacity
                        onPress={() => applyDetectedChannel(channel, 'lidar')}
                        className="bg-dimos-yellow/20 px-2 py-1 rounded">
                        <Text className="text-dimos-yellow text-xs">Lidar</Text>
                      </TouchableOpacity>
                      <TouchableOpacity
                        onPress={() => applyDetectedChannel(channel, 'control')}
                        className="bg-dimos-yellow/20 px-2 py-1 rounded">
                        <Text className="text-dimos-yellow text-xs">Ctrl</Text>
                      </TouchableOpacity>
                    </View>
                  </View>
                ))}
              </ScrollView>
            </View>
          )}
        </View>

        {/* LCM Channels */}
        <View className="mb-6">
          <Text className="text-dimos-yellow text-lg font-bold mb-3">
            LCM Channel Names
          </Text>

          <Text className="text-dimos-yellow text-sm mb-1">Video Channel:</Text>
          <TextInput
            className="bg-white rounded-lg p-3 text-base mb-3 font-mono"
            value={settings.videoChannel}
            onChangeText={(text) => setSettings(prev => ({...prev, videoChannel: text}))}
            placeholder="UNITREE_VIDEO"
            autoCapitalize="characters"
          />

          <Text className="text-dimos-yellow text-sm mb-1">Lidar Channel:</Text>
          <TextInput
            className="bg-white rounded-lg p-3 text-base mb-3 font-mono"
            value={settings.lidarChannel}
            onChangeText={(text) => setSettings(prev => ({...prev, lidarChannel: text}))}
            placeholder="UNITREE_LIDAR"
            autoCapitalize="characters"
          />

          <Text className="text-dimos-yellow text-sm mb-1">Control Channel:</Text>
          <TextInput
            className="bg-white rounded-lg p-3 text-base mb-3 font-mono"
            value={settings.controlChannel}
            onChangeText={(text) => setSettings(prev => ({...prev, controlChannel: text}))}
            placeholder="UNITREE_CONTROL"
            autoCapitalize="characters"
          />
        </View>

        {/* Info Box */}
        <View className="mb-6 p-4 bg-dimos-yellow/10 rounded-lg">
          <Text className="text-dimos-yellow text-xs mb-2">
            💡 <Text className="font-bold">Tip:</Text> Use the "Detect LCM Channels" button to automatically
            find active channels on your robot network.
          </Text>
          <Text className="text-dimos-yellow text-xs">
            Channel names are case-sensitive and must match exactly what your robot publishes.
          </Text>
        </View>

        {/* Save Button */}
        <View className="mb-6">
          <TouchableOpacity
            className={`bg-dimos-yellow rounded-lg p-4 items-center ${isSaving ? 'opacity-60' : ''}`}
            onPress={handleSave}
            disabled={isSaving}>
            {isSaving ? (
              <ActivityIndicator color="#0016B1" />
            ) : (
              <Text className="text-dimos-blue text-lg font-bold">💾 Save Settings</Text>
            )}
          </TouchableOpacity>
        </View>
      </ScrollView>
    </SafeAreaView>
  );
};

export default SettingsScreen;
