import React, {useState, useEffect, useCallback} from 'react';
import LoadingScreen from '../screens/LoadingScreen';
import HomeScreen from '../screens/HomeScreen';
import SettingsScreen, {BridgeSettings} from '../screens/SettingsScreen';
import RobotStreamScreen from '../screens/RobotStreamScreen';

type Screen = 'loading' | 'home' | 'settings' | 'robotStream';

const AppNavigator: React.FC = () => {
  const [currentScreen, setCurrentScreen] = useState<Screen>('loading');
  const [bridgeSettings, setBridgeSettings] = useState<BridgeSettings>({
    serverIP: 'localhost',
    serverPort: '5555',
    videoChannel: 'UNITREE_VIDEO',
    lidarChannel: 'UNITREE_LIDAR',
    controlChannel: 'UNITREE_CONTROL',
    lcmUrl: 'udpm://239.255.76.67:7667?ttl=1',
  });

  useEffect(() => {
    const timer = setTimeout(() => setCurrentScreen('home'), 1200);
    return () => clearTimeout(timer);
  }, []);

  const navigateToSettings = useCallback(() => {
    setCurrentScreen('settings');
  }, []);

  const navigateToHome = useCallback(() => {
    setCurrentScreen('home');
  }, []);

  const navigateToRobotStream = useCallback(() => {
    setCurrentScreen('robotStream');
  }, []);

  const handleSettingsSave = useCallback((settings: BridgeSettings) => {
    setBridgeSettings(settings);
    setCurrentScreen('home');
  }, []);

  if (currentScreen === 'loading') {
    return <LoadingScreen />;
  }

  if (currentScreen === 'settings') {
    return <SettingsScreen onBack={navigateToHome} onSave={handleSettingsSave} />;
  }

  if (currentScreen === 'robotStream') {
    return <RobotStreamScreen onBack={navigateToHome} bridgeSettings={bridgeSettings} />;
  }

  return <HomeScreen onNavigateToSettings={navigateToSettings} onNavigateToRobotStream={navigateToRobotStream} />;
};

export default AppNavigator;
