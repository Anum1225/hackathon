import React from 'react';
import Link from '@docusaurus/Link';

export const PersonalizationButton = () => {
    return (
        <button
            className="button button--secondary button--sm margin-right--sm icon-text"
            onClick={() => alert('Personalization settings coming soon!')}
            style={{ display: 'inline-flex', alignItems: 'center', gap: '6px' }}
        >
            <span>⚙️</span> Personalization
        </button>
    );
};

export const UrduTranslationButton = () => {
    const [isUrdu, setIsUrdu] = React.useState(false);

    return (
        <button
            className="button button--primary button--sm icon-text"
            onClick={() => {
                setIsUrdu(!isUrdu);
                alert(isUrdu ? 'Switched to English' : 'Switched to Urdu (Demo)');
            }}
            style={{ display: 'inline-flex', alignItems: 'center', gap: '6px' }}
        >
            <span>🌐</span> {isUrdu ? 'English' : 'Urdu / اردو'}
        </button>
    );
};

export const ButtonContainer = ({ children }) => (
    <div style={{ marginBottom: '1.5rem', display: 'flex', gap: '0.5rem' }}>
        {children}
    </div>
);
