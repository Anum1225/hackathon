
import React from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';
import { Brain, Bot, Zap, Cpu, Code2, Globe } from 'lucide-react';

type FeatureItem = {
    title: string;
    Icon: React.ElementType;
    description: React.ReactNode;
    color: string;
    span?: string; // For bento grid spanning
};

const FeatureList: FeatureItem[] = [
    {
        title: 'Digital Brain',
        Icon: Brain,
        color: '#7c3aed',
        description: (
            <>
                Master <strong>simulated intelligence</strong> via Deep Learning & LLMs training in virtual environments.
            </>
        ),
        span: 'col-span-2'
    },
    {
        title: 'Physical Body',
        Icon: Bot,
        color: '#06b6d4',
        description: (
            <>
                Understand <strong>mechatronics</strong> and kinematics.
            </>
        ),
    },
    {
        title: 'Sim2Real Gap',
        Icon: Zap,
        color: '#f59e0b',
        description: (
            <>
                Deploy policies directly to hardware with <strong>zero-shot transfer</strong>.
            </>
        ),
    },
    {
        title: 'ROS 2 Native',
        Icon: Cpu,
        color: '#10b981',
        description: (
            <>
                Built on the industry standard middleware for robotics.
            </>
        ),
    },
    {
        title: 'Open Source',
        Icon: Code2,
        color: '#ec4899',
        description: (
            <>
                Entirely open curriculum and codebase.
            </>
        ),
    },
    {
        title: 'Global Community',
        Icon: Globe,
        color: '#3b82f6',
        description: (
            <>
                Join thousands of students building the future.
            </>
        ),
        span: 'col-span-2'
    },
];

function Feature({ title, Icon, description, color, span }: FeatureItem) {
    return (
        <div className={clsx('glass-card', span ? styles.spanTwo : '')} style={{ position: 'relative', overflow: 'hidden' }}>
            <div style={{
                position: 'absolute',
                top: '-20px',
                right: '-20px',
                opacity: 0.1,
                transform: 'rotate(15deg)'
            }}>
                <Icon size={150} color={color} />
            </div>

            <div style={{ display: 'flex', alignItems: 'center', gap: '12px', marginBottom: '16px' }}>
                <div style={{
                    background: `${color}20`,
                    padding: '10px',
                    borderRadius: '12px',
                    display: 'flex'
                }}>
                    <Icon size={24} color={color} />
                </div>
                <Heading as="h3" style={{ margin: 0, fontSize: '1.25rem' }}>{title}</Heading>
            </div>
            <p style={{ margin: 0, opacity: 0.8, lineHeight: '1.6' }}>{description}</p>
        </div>
    );
}

export default function HomepageFeatures(): JSX.Element {
    return (
        <section className={styles.features}>
            <div className="container">
                <div className="bento-grid">
                    {FeatureList.map((props, idx) => (
                        <Feature key={idx} {...props} />
                    ))}
                </div>
            </div>
        </section>
    );
}
