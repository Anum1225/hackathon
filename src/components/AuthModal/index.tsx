
import React, { useState, useEffect } from 'react';
import {
    X, ArrowRight, CheckCircle,
    Code, Cpu, Boxes, BookOpen, // Roles
    Zap, Laptop, Smartphone, Cloud, // Hardware
    GraduationCap, Briefcase, Trophy, Rocket // Experience
} from 'lucide-react';
import styles from './styles.module.css';

interface AuthModalProps {
    isOpen: boolean;
    onClose: () => void;
    initialView?: 'login' | 'signup';
}

export default function AuthModal({ isOpen, onClose, initialView = 'login' }: AuthModalProps) {
    const [view, setView] = useState<'login' | 'signup' | 'onboarding'>(initialView);
    const [step, setStep] = useState(1);
    const [selections, setSelections] = useState({
        role: '',
        exp: '',
        hardware: ''
    });

    // Signup Data State
    const [signupData, setSignupData] = useState({ name: '', email: '', password: '' });
    const [loginData, setLoginData] = useState({ email: '', password: '' });
    const [loading, setLoading] = useState(false);
    const [error, setError] = useState('');

    const API_URL = 'http://localhost:8000/api';

    // Reset when opened
    useEffect(() => {
        if (isOpen) {
            setView(initialView);
            setStep(1);
            setSelections({ role: '', exp: '', hardware: '' });
            setSignupData({ name: '', email: '', password: '' });
            setLoginData({ email: '', password: '' });
            setError('');
        }
    }, [isOpen, initialView]);

    const handleLogin = async (e: React.FormEvent) => {
        e.preventDefault();
        setLoading(true);
        setError('');
        try {
            const res = await fetch(`${API_URL}/auth/login`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(loginData)
            });
            const data = await res.json();
            if (!res.ok) throw new Error(data.detail || 'Login failed');

            // Save session
            localStorage.setItem('user_id', data.user_id);
            localStorage.setItem('user_name', data.name);
            localStorage.setItem('user_email', data.email);

            // Reload page to reflect auth state (simple way) or just close
            // Redirect to Profile page
            window.location.href = '/profile';
        } catch (err: any) {
            setError(err.message);
        } finally {
            setLoading(false);
        }
    };

    const handleSignupStep = (e: React.FormEvent) => {
        e.preventDefault();
        // Just move to onboarding, data is already in state
        setView('onboarding');
    };

    const handleFinalize = async () => {
        setLoading(true);
        setError('');
        try {
            // 1. Signup
            const signupRes = await fetch(`${API_URL}/auth/signup`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(signupData)
            });
            const userData = await signupRes.json();
            if (!signupRes.ok) throw new Error(userData.detail || 'Signup failed');

            localStorage.setItem('user_id', userData.user_id);
            localStorage.setItem('user_name', userData.name);
            localStorage.setItem('user_email', userData.email);

            // 2. Create Profile
            const profileData = {
                software_background: `Role: ${selections.role}, Experience: ${selections.exp}`,
                hardware_background: `Hardware: ${selections.hardware}`,
                preferred_language: 'en' // Default for now
            };

            await fetch(`${API_URL}/auth/profile/${userData.user_id}`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(profileData)
            });

            // Redirect to Profile page
            window.location.href = '/profile';
        } catch (err: any) {
            setError(err.message);
            // Go back to signup if error? Or just show error
            alert(`Error: ${err.message}`);
        } finally {
            setLoading(false);
        }
    };

    if (!isOpen) return null;

    return (
        <div className={styles.overlay} onClick={(e) => {
            if (e.target === e.currentTarget) onClose();
        }}>
            <div className={styles.modalCard}>
                <button className={styles.closeButton} onClick={onClose} aria-label="Close">
                    <X size={20} />
                </button>

                {/* --- LOGIN VIEW --- */}
                {view === 'login' && (
                    <div className={styles.wrapperTransition} key="login">
                        <h2 className={styles.title}>Welcome Back</h2>
                        <p className={styles.subtitle}>Sign in to your Physical AI workspace.</p>
                        {error && <p style={{ color: 'red', textAlign: 'center' }}>{error}</p>}

                        <form onSubmit={handleLogin}>
                            <div className={styles.inputGroup}>
                                <label className={styles.label}>Email Address</label>
                                <input
                                    className={styles.input}
                                    type="email"
                                    placeholder="student@example.com"
                                    autoFocus
                                    value={loginData.email}
                                    onChange={e => setLoginData({ ...loginData, email: e.target.value })}
                                    required
                                />
                            </div>
                            <div className={styles.inputGroup}>
                                <label className={styles.label}>Password</label>
                                <input
                                    className={styles.input}
                                    type="password"
                                    placeholder="••••••••"
                                    value={loginData.password}
                                    onChange={e => setLoginData({ ...loginData, password: e.target.value })}
                                    required
                                />
                            </div>
                            <button type="submit" className={styles.primaryBtn} disabled={loading}>
                                {loading ? 'Signing In...' : 'Sign In'}
                            </button>
                        </form>

                        <div style={{ marginTop: '1.5rem', textAlign: 'center', fontSize: '0.9rem' }}>
                            <span style={{ opacity: 0.7 }}>No account?</span> <button className={styles.tbt} onClick={() => setView('signup')}>Create Account</button>
                        </div>
                    </div>
                )}

                {/* --- SIGNUP VIEW --- */}
                {view === 'signup' && (
                    <div className={styles.wrapperTransition} key="signup">
                        <h2 className={styles.title}>Start Learning</h2>
                        <p className={styles.subtitle}>Join the Physical AI revolution today.</p>

                        <form onSubmit={handleSignupStep}>
                            <div className={styles.inputGroup}>
                                <label className={styles.label}>Full Name</label>
                                <input
                                    className={styles.input}
                                    type="text"
                                    placeholder="John Doe"
                                    autoFocus
                                    value={signupData.name}
                                    onChange={e => setSignupData({ ...signupData, name: e.target.value })}
                                    required
                                />
                            </div>
                            <div className={styles.inputGroup}>
                                <label className={styles.label}>Email Address</label>
                                <input
                                    className={styles.input}
                                    type="email"
                                    placeholder="student@example.com"
                                    value={signupData.email}
                                    onChange={e => setSignupData({ ...signupData, email: e.target.value })}
                                    required
                                />
                            </div>
                            <div className={styles.inputGroup}>
                                <label className={styles.label}>Password</label>
                                <input
                                    className={styles.input}
                                    type="password"
                                    placeholder="Create robust password"
                                    value={signupData.password}
                                    onChange={e => setSignupData({ ...signupData, password: e.target.value })}
                                    required
                                />
                            </div>
                            <button type="submit" className={styles.primaryBtn}>
                                Continue <ArrowRight size={18} />
                            </button>
                        </form>

                        <div style={{ marginTop: '1.5rem', textAlign: 'center', fontSize: '0.9rem' }}>
                            <span style={{ opacity: 0.7 }}>Already have an account?</span> <button className={styles.tbt} onClick={() => setView('login')}>Sign In</button>
                        </div>
                    </div>
                )}

                {/* --- ONBOARDING VIEW (4 Options Grid) --- */}
                {view === 'onboarding' && (
                    <div className={styles.wrapperTransition} key={`onboarding-${step}`}>

                        {/* Progress Bar */}
                        <div style={{ display: 'flex', justifyContent: 'center', marginBottom: '2rem', gap: '8px' }}>
                            {[1, 2, 3, 4].map(s => (
                                <div key={s} style={{
                                    width: step === s ? '32px' : '12px', height: '4px', borderRadius: '2px',
                                    background: step >= s ? 'linear-gradient(90deg, #7c3aed, #06b6d4)' : 'rgba(128,128,128,0.2)',
                                    transition: 'all 0.4s'
                                }} />
                            ))}
                        </div>

                        {/* STEP 1: ROLE */}
                        {step === 1 && (
                            <>
                                <h2 className={styles.title}>Your Role</h2>
                                <p className={styles.subtitle}>Tailors content to your background.</p>
                                <div className={styles.grid2x2}>
                                    <div className={`${styles.optionCard} ${selections.role === 'soft' ? styles.selected : ''}`} onClick={() => setSelections({ ...selections, role: 'soft' })}>
                                        <Code size={28} color="#06b6d4" />
                                        <span className={styles.cardTitle}>Software</span>
                                        <span className={styles.cardDesc}>ROS 2 Stack</span>
                                    </div>
                                    <div className={`${styles.optionCard} ${selections.role === 'hard' ? styles.selected : ''}`} onClick={() => setSelections({ ...selections, role: 'hard' })}>
                                        <Cpu size={28} color="#a78bfa" />
                                        <span className={styles.cardTitle}>Hardware</span>
                                        <span className={styles.cardDesc}>PCB & Sensors</span>
                                    </div>
                                    <div className={`${styles.optionCard} ${selections.role === 'research' ? styles.selected : ''}`} onClick={() => setSelections({ ...selections, role: 'research' })}>
                                        <Boxes size={28} color="#f472b6" />
                                        <span className={styles.cardTitle}>Researcher</span>
                                        <span className={styles.cardDesc}>ML Models</span>
                                    </div>
                                    <div className={`${styles.optionCard} ${selections.role === 'student' ? styles.selected : ''}`} onClick={() => setSelections({ ...selections, role: 'student' })}>
                                        <BookOpen size={28} color="#10b981" />
                                        <span className={styles.cardTitle}>Student</span>
                                        <span className={styles.cardDesc}>Learning</span>
                                    </div>
                                </div>
                                <button className={styles.primaryBtn} disabled={!selections.role} onClick={() => setStep(2)}>Continue <ArrowRight size={18} /></button>
                            </>
                        )}

                        {/* STEP 2: EXPERIENCE */}
                        {step === 2 && (
                            <>
                                <h2 className={styles.title}>Experience</h2>
                                <p className={styles.subtitle}>Adjusts the difficulty curve.</p>
                                <div className={styles.grid2x2}>
                                    <div className={`${styles.optionCard} ${selections.exp === 'new' ? styles.selected : ''}`} onClick={() => setSelections({ ...selections, exp: 'new' })}>
                                        <GraduationCap size={28} color="#fbbf24" />
                                        <span className={styles.cardTitle}>Beginner</span>
                                        <span className={styles.cardDesc}>New to Robotics</span>
                                    </div>
                                    <div className={`${styles.optionCard} ${selections.exp === 'mid' ? styles.selected : ''}`} onClick={() => setSelections({ ...selections, exp: 'mid' })}>
                                        <Briefcase size={28} color="#60a5fa" />
                                        <span className={styles.cardTitle}>Intermediate</span>
                                        <span className={styles.cardDesc}>Used ROS 1</span>
                                    </div>
                                    <div className={`${styles.optionCard} ${selections.exp === 'pro' ? styles.selected : ''}`} onClick={() => setSelections({ ...selections, exp: 'pro' })}>
                                        <Rocket size={28} color="#f87171" />
                                        <span className={styles.cardTitle}>Advanced</span>
                                        <span className={styles.cardDesc}>ROS 2 Native</span>
                                    </div>
                                    <div className={`${styles.optionCard} ${selections.exp === 'expert' ? styles.selected : ''}`} onClick={() => setSelections({ ...selections, exp: 'expert' })}>
                                        <Trophy size={28} color="#a78bfa" />
                                        <span className={styles.cardTitle}>Expert</span>
                                        <span className={styles.cardDesc}>Contributor</span>
                                    </div>
                                </div>
                                <button className={styles.primaryBtn} disabled={!selections.exp} onClick={() => setStep(3)}>Continue <ArrowRight size={18} /></button>
                            </>
                        )}

                        {/* STEP 3: HARDWARE */}
                        {step === 3 && (
                            <>
                                <h2 className={styles.title}>Compute Power</h2>
                                <p className={styles.subtitle}>Optimizes simulation settings.</p>
                                <div className={styles.grid2x2}>
                                    <div className={`${styles.optionCard} ${selections.hardware === 'high' ? styles.selected : ''}`} onClick={() => setSelections({ ...selections, hardware: 'high' })}>
                                        <Zap size={28} color="#10b981" />
                                        <span className={styles.cardTitle}>High End</span>
                                        <span className={styles.cardDesc}>RTX 4090/80</span>
                                    </div>
                                    <div className={`${styles.optionCard} ${selections.hardware === 'mid' ? styles.selected : ''}`} onClick={() => setSelections({ ...selections, hardware: 'mid' })}>
                                        <Laptop size={28} color="#60a5fa" />
                                        <span className={styles.cardTitle}>Mid Range</span>
                                        <span className={styles.cardDesc}>RTX 3060/70</span>
                                    </div>
                                    <div className={`${styles.optionCard} ${selections.hardware === 'mac' ? styles.selected : ''}`} onClick={() => setSelections({ ...selections, hardware: 'mac' })}>
                                        <Smartphone size={28} color="#94a3b8" />
                                        <span className={styles.cardTitle}>Apple/Mac</span>
                                        <span className={styles.cardDesc}>M1/M2/M3</span>
                                    </div>
                                    <div className={`${styles.optionCard} ${selections.hardware === 'cloud' ? styles.selected : ''}`} onClick={() => setSelections({ ...selections, hardware: 'cloud' })}>
                                        <Cloud size={28} color="#f472b6" />
                                        <span className={styles.cardTitle}>Cloud / CPU</span>
                                        <span className={styles.cardDesc}>AWS / Azure</span>
                                    </div>
                                </div>
                                <button className={styles.primaryBtn} disabled={!selections.hardware} onClick={() => setStep(4)}>Finalize <ArrowRight size={18} /></button>
                            </>
                        )}

                        {/* STEP 4: SUCCESS */}
                        {step === 4 && (
                            <div style={{ textAlign: 'center', padding: '1rem 0', animation: 'fadeIn 0.5s' }}>
                                <div style={{
                                    width: '80px', height: '80px', background: 'linear-gradient(135deg, #7c3aed 0%, #06b6d4 100%)',
                                    borderRadius: '50%', display: 'flex', alignItems: 'center', justifyContent: 'center',
                                    margin: '0 auto 2rem auto', boxShadow: '0 0 40px rgba(124, 58, 237, 0.5)'
                                }}>
                                    <CheckCircle size={40} color="white" />
                                </div>
                                <h2 className={styles.title}>We're Ready!</h2>
                                <p className={styles.subtitle} style={{ marginBottom: '2.5rem' }}>
                                    Your tailored learning path is generated.
                                </p>
                                <button className={styles.primaryBtn} onClick={handleFinalize} disabled={loading}>
                                    {loading ? 'Setting up...' : 'Launch Dashboard'}
                                </button>
                            </div>
                        )}
                    </div>
                )}

            </div>
        </div>
    );
}
