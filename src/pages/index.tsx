import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';
import { ArrowRight, Github, TrendingUp, Users, Award, Code2, Box, Cpu, Layers, CheckCircle2 } from 'lucide-react';

import styles from './index.module.css';

// Marquee Logos
const Technologies = [
    { name: 'ROS 2', icon: Box },
    { name: 'Gazebo', icon: Layers },
    { name: 'NVIDIA Isaac', icon: Cpu },
    { name: 'PyTorch', icon: Code2 },
    { name: 'React', icon: Box },
    { name: 'TypeScript', icon: Code2 },
];

function HomepageHeader() {
    const { siteConfig } = useDocusaurusContext();
    return (
        <header className={clsx('hero', styles.heroBanner)} style={{ overflow: 'hidden', position: 'relative', background: 'transparent' }}>
            {/* Ambient Glows */}
            <div className="ambient-glow glow-top-right" />
            <div className="ambient-glow glow-bottom-left" />

            <div className="container" style={{ position: 'relative', zIndex: 1 }}>
                <div className="row" style={{ alignItems: 'center', minHeight: '650px' }}>
                    <div className="col col--6" style={{ textAlign: 'left' }}>

                        <div className="animate-fade-in">
                            <span style={{
                                background: 'rgba(124, 58, 237, 0.1)',
                                color: '#a78bfa',
                                padding: '8px 20px',
                                borderRadius: '24px',
                                fontSize: '0.9rem',
                                fontWeight: '600',
                                border: '1px solid rgba(124, 58, 237, 0.3)',
                                boxShadow: '0 0 20px rgba(124, 58, 237, 0.15)'
                            }}>
                                ✨ Physical AI Textbook
                            </span>
                        </div>

                        <Heading as="h1" className={clsx('hero__title', styles.heroTitle, 'animate-fade-in')} style={{ marginTop: '2rem', fontSize: '4.5rem', lineHeight: '1.1' }}>
                            <span className="gradient-text">{siteConfig.title}</span>
                        </Heading>
                        <p className={clsx('hero__subtitle', styles.heroSubtitle, 'animate-fade-in')} style={{ animationDelay: '0.2s', fontSize: '1.4rem', opacity: 0.85, maxWidth: '540px' }}>
                            {siteConfig.tagline}
                        </p>
                        <div className={clsx(styles.buttons, 'animate-fade-in')} style={{ animationDelay: '0.4s', justifyContent: 'flex-start', marginTop: '2.5rem' }}>
                            <Link
                                className="button button--lg hero-button"
                                to="/docs/intro"
                                style={{
                                    color: 'white',
                                    display: 'flex',
                                    alignItems: 'center',
                                    gap: '10px',
                                    padding: '18px 36px',
                                    fontSize: '1.25rem',
                                    fontWeight: 'bold',
                                    borderRadius: '16px'
                                }}>
                                Start Learning <ArrowRight size={22} />
                            </Link>
                            <Link
                                className="button button--secondary button--lg glass-card"
                                to="https://github.com/panaversity/physical-ai-book"
                                style={{
                                    marginLeft: '20px',
                                    padding: '18px 36px',
                                    fontSize: '1.25rem',
                                    display: 'flex',
                                    alignItems: 'center',
                                    gap: '10px',
                                    borderRadius: '16px',
                                    background: 'rgba(255,255,255,0.03)',
                                    borderColor: 'rgba(255,255,255,0.08)',
                                    backdropFilter: 'blur(4px)'
                                }}>
                                <Github size={22} /> GitHub
                            </Link>
                        </div>

                        {/* Stats Row */}
                        <div style={{ marginTop: '5rem', display: 'flex', gap: '4rem', borderTop: '1px solid rgba(255,255,255,0.08)', paddingTop: '2.5rem' }} className="animate-fade-in">
                            <div>
                                <h3 className="gradient-text" style={{ margin: 0, fontSize: '2.5rem', fontWeight: 800 }}>12+</h3>
                                <p style={{ margin: 0, opacity: 0.6, fontSize: '1rem', fontWeight: 500 }}>Weeks of Content</p>
                            </div>
                            <div>
                                <h3 className="gradient-text" style={{ margin: 0, fontSize: '2.5rem', fontWeight: 800 }}>0$</h3>
                                <p style={{ margin: 0, opacity: 0.6, fontSize: '1rem', fontWeight: 500 }}>Hardware Cost</p>
                            </div>
                            <div>
                                <h3 className="gradient-text" style={{ margin: 0, fontSize: '2.5rem', fontWeight: 800 }}>100%</h3>
                                <p style={{ margin: 0, opacity: 0.6, fontSize: '1rem', fontWeight: 500 }}>Open Source</p>
                            </div>
                        </div>
                    </div>

                    <div className="col col--6">
                        <div className="animate-float" style={{ position: 'relative', marginTop: '-2rem' }}>
                            {/* Image Glow */}
                            <div style={{
                                position: 'absolute',
                                top: '50%',
                                left: '50%',
                                transform: 'translate(-50%, -50%)',
                                width: '90%',
                                height: '90%',
                                background: 'radial-gradient(circle, #7c3aed 0%, transparent 60%)',
                                opacity: 0.3,
                                filter: 'blur(60px)',
                                zIndex: -1
                            }} />
                            <img
                                src={require('@site/static/img/module-intro.png').default}
                                alt="Physical AI Intro"
                                style={{
                                    width: '100%',
                                    height: 'auto',
                                    borderRadius: '32px',
                                    border: '1px solid rgba(255,255,255,0.08)',
                                    boxShadow: '0 40px 80px -20px rgba(0,0,0,0.6), 0 0 0 1px rgba(255,255,255,0.05)'
                                }}
                            />
                        </div>
                    </div>
                </div>
            </div>
        </header>
    );
}

function LearningPath() {
    const steps = [
        { title: 'Foundations', desc: 'Python, Linux, and ROS 2 Basics', color: '#10b981' },
        { title: 'Simulation', desc: 'Mastering Gazebo & Isaac Sim', color: '#3b82f6' },
        { title: 'Intelligence', desc: 'Deep Learning & Reinforcement Learning', color: '#7c3aed' },
        { title: 'Real World', desc: 'Sim2Real Transfer & Deployment', color: '#f59e0b' }
    ];

    return (
        <section style={{ padding: '6rem 0' }}>
            <div className="container">
                <div className="text--center" style={{ marginBottom: '4rem' }}>
                    <span style={{ color: '#8b5cf6', fontWeight: 'bold', letterSpacing: '1px', textTransform: 'uppercase', fontSize: '0.9rem' }}>Curriculum</span>
                    <Heading as="h2" style={{ fontSize: '3rem', marginTop: '0.5rem' }}>Your Learning Path</Heading>
                </div>

                <div style={{ position: 'relative', display: 'flex', justifyContent: 'space-between', maxWidth: '1000px', margin: '0 auto' }}>
                    {/* Line */}
                    <div style={{ position: 'absolute', top: '24px', left: '0', right: '0', height: '2px', background: 'rgba(255,255,255,0.1)', zIndex: 0 }} />

                    {steps.map((step, idx) => (
                        <div key={idx} style={{ position: 'relative', zIndex: 1, textAlign: 'center', width: '200px' }}>
                            <div style={{
                                width: '50px',
                                height: '50px',
                                background: '#0f172a',
                                border: `2px solid ${step.color}`,
                                borderRadius: '50%',
                                margin: '0 auto 1.5rem',
                                display: 'flex',
                                alignItems: 'center',
                                justifyContent: 'center',
                                boxShadow: `0 0 20px ${step.color}40`
                            }}>
                                <CheckCircle2 size={24} color={step.color} />
                            </div>
                            <h3 style={{ fontSize: '1.25rem', marginBottom: '0.5rem' }}>{step.title}</h3>
                            <p style={{ fontSize: '0.9rem', opacity: 0.7 }}>{step.desc}</p>
                        </div>
                    ))}
                </div>
            </div>
        </section>
    );
}

function IndustryImpact() {
    return (
        <section style={{ padding: '8rem 0', background: 'linear-gradient(180deg, transparent 0%, rgba(124, 58, 237, 0.05) 100%)' }}>
            <div className="container">
                <div className="row" style={{ alignItems: 'center' }}>
                    <div className="col col--5">
                        <span style={{ color: '#06b6d4', fontWeight: 'bold', letterSpacing: '1px', textTransform: 'uppercase', fontSize: '0.9rem' }}>Why Now?</span>
                        <Heading as="h2" style={{ fontSize: '3.5rem', marginTop: '0.5rem', lineHeight: '1.1' }}>The Age of <br /><span className="gradient-text">Embodied AI</span></Heading>
                        <p style={{ margin: '2rem 0', opacity: 0.8, fontSize: '1.2rem', lineHeight: '1.7' }}>
                            The next frontier of AI isn't just generating text—it's interacting with the physical world.
                            We are standing at the precipice of a trillion-dollar shift in how machines work.
                        </p>
                        <Link className="button button--primary button--lg hero-button" to="/docs/intro" style={{ borderRadius: '12px' }}>
                            Read the Manifesto
                        </Link>
                    </div>

                    <div className="col col--7">
                        <div className="row">
                            <div className="col col--6" style={{ marginBottom: '24px' }}>
                                <div className="glass-card" style={{ height: '100%', background: 'rgba(16, 185, 129, 0.05)', borderColor: 'rgba(16, 185, 129, 0.2)' }}>
                                    <TrendingUp size={40} color="#10b981" style={{ marginBottom: '1rem' }} />
                                    <h3 style={{ fontSize: '1.5rem' }}>Trillion $ Market</h3>
                                    <p style={{ opacity: 0.7 }}>Projected growth for humanoid robotics by 2030.</p>
                                </div>
                            </div>
                            <div className="col col--6" style={{ marginBottom: '24px' }}>
                                <div className="glass-card" style={{ height: '100%', background: 'rgba(245, 158, 11, 0.05)', borderColor: 'rgba(245, 158, 11, 0.2)' }}>
                                    <Users size={40} color="#f59e0b" style={{ marginBottom: '1rem' }} />
                                    <h3 style={{ fontSize: '1.5rem' }}>Labor Gap</h3>
                                    <p style={{ opacity: 0.7 }}>Filling 85M+ unfilled jobs in the global supply chain.</p>
                                </div>
                            </div>
                            <div className="col col--12">
                                <div className="glass-card" style={{ background: 'rgba(59, 130, 246, 0.05)', borderColor: 'rgba(59, 130, 246, 0.2)' }}>
                                    <div style={{ display: 'flex', gap: '1.5rem', alignItems: 'center' }}>
                                        <Award size={40} color="#3b82f6" />
                                        <div>
                                            <h3 style={{ fontSize: '1.5rem', margin: 0 }}>The Path to AGI</h3>
                                            <p style={{ opacity: 0.7, margin: 0 }}>Physical grounding is the missing link for true intelligence.</p>
                                        </div>
                                    </div>
                                </div>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </section>
    );
}

function MarqueeSection() {
    return (
        <section style={{ padding: '3rem 0', borderTop: '1px solid rgba(255,255,255,0.05)', borderBottom: '1px solid rgba(255,255,255,0.05)', background: 'rgba(0,0,0,0.2)' }}>
            <div className="marquee-container">
                <div className="marquee-content">
                    {/* Double the list for infinite scroll illusion */}
                    {[...Technologies, ...Technologies, ...Technologies, ...Technologies].map((tech, idx) => (
                        <div key={idx} style={{ display: 'flex', alignItems: 'center', gap: '12px', opacity: 0.5, filter: 'grayscale(100%)', transition: 'all 0.3s' }}>
                            <tech.icon size={28} />
                            <span style={{ fontWeight: '600', fontSize: '1.3rem' }}>{tech.name}</span>
                        </div>
                    ))}
                </div>
            </div>
        </section>
    );
}

export default function Home(): React.JSX.Element {
    const { siteConfig } = useDocusaurusContext();
    return (
        <Layout
            title={`Hello from ${siteConfig.title}`}
            description="Learn Physical AI and Humanoid Robotics with hands-on projects.">
            <HomepageHeader />
            <main>
                <MarqueeSection />
                <HomepageFeatures />
                <LearningPath />
                <IndustryImpact />
            </main>
        </Layout>
    );
}
