import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import styles from './Onboarding.module.css';

const API_URL = process.env.DOCUSAURUS_CUSTOM_API_URL || 'http://localhost:8000/api/v1';

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
  const [step, setStep] = useState(0);
  const [formData, setFormData] = useState({
    hardware: [],
    software: [],
    experienceLevel: '',
    interests: [],
  });
  const [user, setUser] = useState(null);

  useEffect(() => {
    // Check for existing session
    const savedUser = localStorage.getItem('physical_ai_user');
    if (savedUser) {
      setUser(JSON.parse(savedUser));
      onComplete();
    }
  }, [onComplete]);

  const handleHardwareToggle = (id) => {
    setFormData((prev) => ({
      ...prev,
      hardware: prev.hardware.includes(id)
        ? prev.hardware.filter((h) => h !== id)
        : [...prev.hardware, id],
    }));
  };

  const handleSoftwareToggle = (id) => {
    setFormData((prev) => ({
      ...prev,
      software: prev.software.includes(id)
        ? prev.software.filter((s) => s !== id)
        : [...prev.software, id],
    }));
  };

  const handleExperienceSelect = (id) => {
    setFormData((prev) => ({
      ...prev,
      experienceLevel: id,
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

  const handleSubmit = async () => {
    // Get Better-Auth user info (this would come from actual auth)
    const authUser = {
      id: 'temp-auth-id',
      email: 'user@example.com',
      name: 'Student',
    };

    // Save to localStorage for now
    const userData = {
      ...authUser,
      ...formData,
      onboardingCompleted: true,
    };
    localStorage.setItem('physical_ai_user', JSON.stringify(userData));
    setUser(userData);
    onComplete();
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
    if (step < 3) {
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

  if (user) {
    return null;
  }

  return (
    <div className={styles.overlay}>
      <div className={styles.modal}>
        <div className={styles.progressBar}>
          <div
            className={styles.progressFill}
            style={{ width: `${((step + 1) / 4) * 100}%` }}
          />
        </div>

        {step === 0 && (
          <>
            <h2>Hardware Experience</h2>
            <p>What hardware and robotics platforms have you worked with?</p>
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
          </>
        )}

        {step === 1 && (
          <>
            <h2>Software Experience</h2>
            <p>What programming languages and ML frameworks do you know?</p>
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
          </>
        )}

        {step === 2 && (
          <>
            <h2>Experience Level</h2>
            <p>How would you describe your overall experience?</p>
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
          </>
        )}

        {step === 3 && (
          <>
            <h2>Your Interests</h2>
            <p>What topics would you like to explore in this course?</p>
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
          </>
        )}

        <div className={styles.buttonRow}>
          {step > 0 && (
            <button className={styles.backBtn} onClick={prevStep}>
              Back
            </button>
          )}
          <button
            className={styles.nextBtn}
            onClick={nextStep}
            disabled={!canProceed()}
          >
            {step === 3 ? 'Get Started' : 'Continue'}
          </button>
        </div>
      </div>
    </div>
  );
}

export default Onboarding;
