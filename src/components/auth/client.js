/**
 * Auth Client for Physical AI Textbook
 *
 * Implements Google OAuth with session management
 * using the FastAPI backend.
 */

const API_URL = process.env.BACKEND_URL || 'http://localhost:8000';

/**
 * Sign in with Google - redirects to OAuth flow
 */
export async function signInWithGoogle() {
  try {
    const response = await fetch(`${API_URL}/api/v1/auth/google`);
    const data = await response.json();

    if (data.auth_url) {
      window.location.href = data.auth_url;
    }
  } catch (error) {
    console.error('Failed to initiate Google auth:', error);
    throw error;
  }
}

/**
 * Handle OAuth callback from backend
 */
export async function handleAuthCallback(accessToken: string, refreshToken: string) {
  localStorage.setItem('access_token', accessToken);
  localStorage.setItem('refresh_token', refreshToken);
}

/**
 * Get current access token
 */
export function getAccessToken(): string | null {
  return localStorage.getItem('access_token');
}

/**
 * Check if user is authenticated
 */
export function isAuthenticated(): boolean {
  return !!getAccessToken();
}

/**
 * Refresh session token
 */
export async function refreshSession(): Promise<boolean> {
  const refreshToken = localStorage.getItem('refresh_token');
  if (!refreshToken) return false;

  try {
    const response = await fetch(`${API_URL}/api/v1/auth/refresh`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ refresh_token: refreshToken }),
    });

    if (response.ok) {
      const data = await response.json();
      localStorage.setItem('access_token', data.access_token);
      localStorage.setItem('refresh_token', data.refresh_token);
      return true;
    }
  } catch (error) {
    console.error('Failed to refresh token:', error);
  }
  return false;
}

/**
 * Sign out user
 */
export async function signOut() {
  try {
    const accessToken = getAccessToken();
    if (accessToken) {
      await fetch(`${API_URL}/api/v1/auth/logout`, {
        method: 'POST',
        headers: { 'Authorization': `Bearer ${accessToken}` },
      });
    }
  } catch (error) {
    console.error('Error during sign out:', error);
  } finally {
    localStorage.removeItem('access_token');
    localStorage.removeItem('refresh_token');
  }
}

/**
 * Get current user profile
 */
export async function getCurrentUser(): Promise<{ id: string; email: string; name: string; profile: Record<string, unknown> } | null> {
  const accessToken = getAccessToken();
  if (!accessToken) return null;

  try {
    const response = await fetch(`${API_URL}/api/v1/auth/me`, {
      headers: { 'Authorization': `Bearer ${accessToken}` },
    });
    if (response.ok) {
      return await response.json();
    }
  } catch (error) {
    console.error('Failed to get user:', error);
  }
  return null;
}

/**
 * Submit onboarding data
 */
export async function submitOnboarding(data: {
  experience_level: string;
  hardware_background: string[];
  software_background: string[];
  learning_goals: string[];
  preferred_code_complexity?: string;
}): Promise<boolean> {
  const accessToken = getAccessToken();
  if (!accessToken) return false;

  try {
    const response = await fetch(`${API_URL}/api/v1/auth/onboarding`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${accessToken}`,
      },
      body: JSON.stringify(data),
    });
    return response.ok;
  } catch (error) {
    console.error('Failed to submit onboarding:', error);
    return false;
  }
}

/**
 * Get onboarding status
 */
export async function getOnboardingStatus(): Promise<{ completed: boolean; experience_level?: string }> {
  const accessToken = getAccessToken();
  if (!accessToken) return { completed: false };

  try {
    const response = await fetch(`${API_URL}/api/v1/auth/onboarding-status`, {
      headers: { 'Authorization': `Bearer ${accessToken}` },
    });
    return await response.json();
  } catch {
    return { completed: false };
  }
}

export default {
  signInWithGoogle,
  handleAuthCallback,
  getAccessToken,
  isAuthenticated,
  refreshSession,
  signOut,
  getCurrentUser,
  submitOnboarding,
  getOnboardingStatus,
};
