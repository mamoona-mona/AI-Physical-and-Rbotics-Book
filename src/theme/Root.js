/**
 * Root layout with ChatWidget and Onboarding
 */
import React, { useState, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import Onboarding from '@site/src/components/Onboarding';
import ChatWidget from '@site/src/components/ChatWidget';

// Context to manage onboarding state
export const OnboardingContext = React.createContext({
  showOnboarding: false,
  completeOnboarding: () => {},
  isOnboardingComplete: false,
});

export default function Root({ children }) {
  const location = useLocation();
  const [showOnboarding, setShowOnboarding] = useState(false);
  const [isOnboardingComplete, setIsOnboardingComplete] = useState(false);

  useEffect(() => {
    // Check if onboarding is completed
    const user = localStorage.getItem('physical_ai_user');
    if (!user) {
      // Show onboarding only on first visit to the home page
      if (location.pathname === '/' || location.pathname === '/index.html') {
        setShowOnboarding(true);
      }
    } else {
      const userData = JSON.parse(user);
      setIsOnboardingComplete(userData.onboardingCompleted || false);
    }
  }, [location.pathname]);

  const completeOnboarding = () => {
    setShowOnboarding(false);
    setIsOnboardingComplete(true);
  };

  return (
    <OnboardingContext.Provider
      value={{
        showOnboarding,
        completeOnboarding,
        isOnboardingComplete,
      }}
    >
      {children}
      <ChatWidget />
      {showOnboarding && <Onboarding onComplete={completeOnboarding} />}
    </OnboardingContext.Provider>
  );
}
