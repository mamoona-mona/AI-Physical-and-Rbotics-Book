/**
 * Onboarding Component - Signup questionnaire for personalization
 */
import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './Onboarding.module.css';

// Onboarding options
const HARDWARE_OPTIONS = [
  { id: 'ros', label: 'ROS/ROS2' },
  { id: 'arduino', label: 'Arduino/Microcontrollers' },
  { id: 'raspberry-pi', label: 'Raspberry Pi' },
  { id: 'actuators', label: 'Actuators & Motors' },
  { id: 'sensors', label: 'Sensors & Vision' },
  { id: '3d-printing', label: '3D Printing & CAD' },
  { id: 'embedded', label: 'Embedded Systems' },
  { id: 'none', label: 'No hardware experience' },
];

const SOFTWARE_OPTIONS = [
  { id: 'python', label: 'Python' },
  { id: 'cpp', label: 'C++' },
  { id: 'pytorch', label: 'PyTorch' },
  { id: 'tensorflow', label: 'TensorFlow' },
  { id: 'ml', label: 'Machine Learning' },
  { id: 'computer-vision', label: 'Computer Vision' },
  { id: 'simulation', label: 'Gazebo/Simulation' },
  { id: 'control', label: 'Control Theory' },
  { id: 'none', label: 'No software experience' },
];

const EXPERIENCE_LEVELS = [
  { id: 'beginner', label: 'Beginner', description: 'New to robotics and AI' },
  { id: 'intermediate', label: 'Intermediate', description: 'Some experience with programming' },
  { id: 'advanced', label: 'Advanced', description: 'Experienced with robotics/ML' },
];

const TOPIC_INTERESTS = [
  { id: 'robotics-basics', label: 'Robotics Fundamentals' },
  { id: 'control-systems', label: 'Control Systems' },
  { id: 'computer-vision', label: 'Computer Vision' },
  { id: 'reinforcement-learning', label: 'Reinforcement Learning' },
  { id: 'sim-to-real', label: 'Sim-to-Real Transfer' },
  { id: 'humanoid-design', label: 'Humanoid Design' },
  { id: 'embodied-ai', label: 'Embodied AI' },
  { id: 'manipulation', label: 'Manipulation & Grasping' },
  { id: 'locomotion', label: 'Locomotion & Navigation' },
];

function Onboarding({ onComplete }) {
  const { siteConfig } = useDocusaurusContext();
  const AUTH_URL = siteConfig.customFields.betterAuthUrl;
  const API_URL = process.env.DOCUSAURUS_CUSTOM_API_URL || 'http://localhost:8000/api/v1';

  const [step, setStep] = useState(0);
  const [formData, setFormData] = useState({
    hardware: [],
    software: [],
    experienceLevel: '',
    interests: [],
    learningGoals: '',
    codeComplexity: 'intermediate',
  });
  const [isLoading, setIsLoading] = useState(false);
  const [user, setUser] = useState(null);

  useEffect(() => {
    // Check for existing session
    const savedUser = localStorage.getItem('physical_ai_user');
    if (savedUser) {
      const userData = JSON.parse(savedUser);
      setUser(userData);
      if (userData.onboardingCompleted) {
        onComplete();
      }
    }
  }, [onComplete]);

  const handleHardwareToggle = (id) => {
    if (id === 'none') {
      setFormData((prev) => ({
        ...prev,
        hardware: prev.hardware.includes('none') ? [] : ['none'],
      }));
      return;
    }
    setFormData((prev) => ({
      ...prev,
      hardware: prev.hardware.filter((h) => h !== 'none')
        .includes(id)
        ? prev.hardware.filter((h) => h !== id)
        : [...prev.hardware.filter((h) => h !== 'none'), id],
    }));
  };

  const handleSoftwareToggle = (id) => {
    if (id === 'none') {
      setFormData((prev) => ({
        ...prev,
        software: prev.software.includes('none') ? [] : ['none'],
      }));
      return;
    }
    setFormData((prev) => ({
      ...prev,
      software: prev.software.filter((s) => s !== 'none')
        .includes(id)
        ? prev.software.filter((s) => s !== id)
        : [...prev.software.filter((s) => s !== 'none'), id],
    }));
  };

  const handleInterestsToggle = (id) => {
    setFormData((prev) => ({
      ...prev,
      interests: prev.interests.includes(id)
        ? prev.interests.filter((i) => i !== id)
        : [...prev.interests, id],
    }));
  };

  const handleExperienceSelect = (id) => {
    setFormData((prev) => ({
      ...prev,
      experienceLevel: id,
    }));
  };

  const handleSubmit = async () => {
    setIsLoading(true);

    try {
      // Save to backend
      const response = await fetch(`${API_URL}/users/onboarding`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          experience_level: formData.experienceLevel,
          hardware_experience: formData.hardware,
          software_experience: formData.software,
          topic_interests: formData.interests,
          learning_goals: formData.learningGoals,
          code_complexity: formData.codeComplexity,
        }),
      });

      if (!response.ok) {
        throw new Error('Failed to save onboarding data');
      }

      const data = await response.json();

      // Save to localStorage
      const userData = {
        ...formData,
        onboardingCompleted: true,
        ui_preferences: data.ui_preferences,
      };
      localStorage.setItem('physical_ai_user', JSON.stringify(userData));
      setUser(userData);
      onComplete();
    } catch (error) {
      console.error('Onboarding error:', error);
      // Still complete onboarding even if backend fails
      const userData = {
        ...formData,
        onboardingCompleted: true,
        ui_preferences: {
          show_advanced_code: formData.experienceLevel !== 'beginner',
          show_math_details: true,
          show_practical_examples: true,
          code_language_preference: formData.software.includes('cpp') ? 'cpp' : 'python',
        },
      };
      localStorage.setItem('physical_ai_user', JSON.stringify(userData));
      onComplete();
    } finally {
      setIsLoading(false);
    }
  };

  const canProceed = () => {
    switch (step) {
      case 0:
        return formData.hardware.length > 0;
      case 1:
        return formData.software.length > 0;
      case 2:
        return formData.experienceLevel !== '';
      case 3:
        return formData.interests.length > 0;
      default:
        return true;
    }
  };

  const nextStep = () => {
    if (step < 4) {
      setStep(step + 1);
    } else {
      handleSubmit();
    }
  };

  const prevStep = () => {
    if (step > 0) {
      setStep(step - 1);
    }
  };

  if (user && user.onboardingCompleted) {
    return null;
  }

  const steps = [
    { title: 'Hardware Experience', subtitle: 'What hardware have you worked with?' },
    { title: 'Software Experience', subtitle: 'What programming skills do you have?' },
    { title: 'Experience Level', subtitle: 'How would you describe your level?' },
    { title: 'Your Interests', subtitle: 'What topics interest you most?' },
    { title: 'Learning Goals', subtitle: 'What do you want to achieve?' },
  ];

  return (
    <div className={styles.overlay}>
      <div className={styles.modal}>
        <div className={styles.progressBar}>
          <div
            className={styles.progressFill}
            style={{ width: `${((step + 1) / steps.length) * 100}%` }}
          />
        </div>

        <div className={styles.stepIndicator}>
          Step {step + 1} of {steps.length}
        </div>

        <h2>{steps[step].title}</h2>
        <p>{steps[step].subtitle}</p>

        {step === 0 && (
          <div className={styles.optionsGrid}>
            {HARDWARE_OPTIONS.map((option) => (
              <button
                key={option.id}
                className={clsx(styles.option, {
                  [styles.selected]: formData.hardware.includes(option.id),
                })}
                onClick={() => handleHardwareToggle(option.id)}
              >
                {option.label}
              </button>
            ))}
          </div>
        )}

        {step === 1 && (
          <div className={styles.optionsGrid}>
            {SOFTWARE_OPTIONS.map((option) => (
              <button
                key={option.id}
                className={clsx(styles.option, {
                  [styles.selected]: formData.software.includes(option.id),
                })}
                onClick={() => handleSoftwareToggle(option.id)}
              >
                {option.label}
              </button>
            ))}
          </div>
        )}

        {step === 2 && (
          <div className={styles.levelsList}>
            {EXPERIENCE_LEVELS.map((level) => (
              <button
                key={level.id}
                className={clsx(styles.levelOption, {
                  [styles.selected]: formData.experienceLevel === level.id,
                })}
                onClick={() => handleExperienceSelect(level.id)}
              >
                <span className={styles.levelLabel}>{level.label}</span>
                <span className={styles.levelDesc}>{level.description}</span>
              </button>
            ))}
          </div>
        )}

        {step === 3 && (
          <div className={styles.optionsGrid}>
            {TOPIC_INTERESTS.map((topic) => (
              <button
                key={topic.id}
                className={clsx(styles.option, {
                  [styles.selected]: formData.interests.includes(topic.id),
                })}
                onClick={() => handleInterestsToggle(topic.id)}
              >
                {topic.label}
              </button>
            ))}
          </div>
        )}

        {step === 4 && (
          <div className={styles.textAreaSection}>
            <label className={styles.textAreaLabel}>
              What are your learning goals? (optional)
            </label>
            <textarea
              className={styles.textArea}
              placeholder="e.g., I want to build a humanoid robot that can walk and manipulate objects..."
              value={formData.learningGoals}
              onChange={(e) => setFormData((prev) => ({
                ...prev,
                learningGoals: e.target.value,
              }))}
              rows={4}
            />
          </div>
        )}

        <div className={styles.buttonRow}>
          {step > 0 && (
            <button className={styles.backBtn} onClick={prevStep} disabled={isLoading}>
              Back
            </button>
          )}
          <button
            className={styles.nextBtn}
            onClick={nextStep}
            disabled={!canProceed() || isLoading}
          >
            {isLoading ? 'Saving...' : step === 4 ? 'Get Started' : 'Continue'}
          </button>
        </div>
      </div>
    </div>
  );
}

export default Onboarding;
