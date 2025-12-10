import React, { useEffect, useState } from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import { User, Mail, Shield, Zap, Code, LogOut } from 'lucide-react';

export default function Profile() {
    const [user, setUser] = useState<{ id: string, name: string, email: string } | null>(null);
    const [profile, setProfile] = useState<any>(null);
    const [loading, setLoading] = useState(true);

    useEffect(() => {
        const id = localStorage.getItem('user_id');
        const name = localStorage.getItem('user_name');
        const email = localStorage.getItem('user_email');

        if (id && name) {
            setUser({ id, name, email: email || '' });
            fetchProfile(id);
        } else {
            setLoading(false);
        }
    }, []);

    const fetchProfile = async (id: string) => {
        try {
            const res = await fetch(`http://localhost:8000/api/auth/profile/${id}`);
            if (res.ok) {
                const data = await res.json();
                setProfile(data);
            }
        } catch (e) {
            console.error("Failed to fetch profile", e);
        } finally {
            setLoading(false);
        }
    };

    const handleLogout = () => {
        localStorage.clear();
        window.location.href = '/';
    };

    if (!user && !loading) {
        return (
            <Layout title="User Profile" description="Your Physical AI Profile">
                <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', minHeight: '60vh' }}>
                    <h1>Sign In Required</h1>
                    <p>Please log in to view your profile.</p>
                    <Link className="button button--primary" to="/">Go Home</Link>
                </div>
            </Layout>
        );
    }

    return (
        <Layout title="User Profile" description="Your Physical AI Profile">
            <div className="container margin-vert--lg">
                <div className="row">
                    <div className="col col--8 col--offset-2">
                        <div className="card shadow--md">
                            <div className="card__header" style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
                                <h2>Your Profile</h2>
                                <button className="button button--outline button--danger button--sm icon-text" onClick={handleLogout}>
                                    <LogOut size={14} style={{ marginRight: 6 }} /> Logout
                                </button>
                            </div>
                            <div className="card__body">

                                <div style={{ display: 'flex', alignItems: 'center', gap: '1rem', marginBottom: '2rem' }}>
                                    <div style={{ width: 80, height: 80, borderRadius: '50%', background: '#e5e7eb', display: 'flex', alignItems: 'center', justifyContent: 'center' }}>
                                        <User size={40} color="#9ca3af" />
                                    </div>
                                    <div>
                                        <h3 style={{ margin: 0 }}>{user?.name}</h3>
                                        <p style={{ margin: 0, opacity: 0.7 }}>{user?.email}</p>
                                    </div>
                                </div>

                                <div className="row">
                                    <div className="col col--6">
                                        <div className="alert alert--secondary">
                                            <div style={{ display: 'flex', alignItems: 'center', gap: '8px', marginBottom: '8px' }}>
                                                <Shield size={18} /> <strong>Account ID</strong>
                                            </div>
                                            <code style={{ wordBreak: 'break-all', fontSize: '0.8rem' }}>{user?.id}</code>
                                        </div>
                                    </div>
                                </div>

                                {profile && (
                                    <div style={{ marginTop: '2rem' }}>
                                        <h3><Zap size={20} style={{ verticalAlign: 'middle' }} /> Onboarding Details</h3>
                                        <hr />
                                        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '1rem' }}>
                                            <div className="card shadow--lw">
                                                <div className="card__body">
                                                    <strong>Software & Experience</strong>
                                                    <p>{profile.software_background}</p>
                                                </div>
                                            </div>
                                            <div className="card shadow--lw">
                                                <div className="card__body">
                                                    <strong>Hardware Setup</strong>
                                                    <p>{profile.hardware_background}</p>
                                                </div>
                                            </div>
                                        </div>
                                    </div>
                                )}

                            </div>
                            <div className="card__footer">
                                <Link className="button button--primary button--block" to="/docs/intro">
                                    Continue Learning
                                </Link>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </Layout>
    );
}
