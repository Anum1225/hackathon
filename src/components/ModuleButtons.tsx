import React from 'react';
import Link from '@docusaurus/Link';

export const PersonalizationButton = () => {
    const [loading, setLoading] = React.useState(false);

    const handlePersonalize = async () => {
        const userId = localStorage.getItem('user_id');
        if (!userId) {
            alert("Please Sign In to use Personlization features.");
            return;
        }

        // Target the wrapper div
        let targetEl = document.getElementById('module-content');
        if (!targetEl) targetEl = document.querySelector('article');

        if (!targetEl) return;

        setLoading(true);
        try {
            const textContent = targetEl.innerText;
            const res = await fetch('http://localhost:8000/api/personalize', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    content: textContent,
                    user_id: userId
                })
            });
            const data = await res.json();
            if (!res.ok) throw new Error(data.detail || 'Personalization failed');

            // Replace content with personalized version
            const personalizedHtml = data.personalized_content
                .replace(/\*\*(.*?)\*\*/g, '<b>$1</b>')
                .replace(/### (.*?)\n/g, '<h3>$1</h3>')
                .replace(/## (.*?)\n/g, '<h2>$1</h2>')
                .replace(/\n/g, '<br/>');

            targetEl.innerHTML = `<div class="theme-doc-markdown markdown">
                <div class="alert alert--info" role="alert">✨ Content Personalized for You</div>
                ${personalizedHtml}
            </div>`;

        } catch (e: any) {
            alert("Error personalizing: " + e.message);
        } finally {
            setLoading(false);
        }
    };

    return (
        <button
            className="button button--secondary button--sm margin-right--sm icon-text"
            onClick={handlePersonalize}
            disabled={loading}
            style={{ display: 'inline-flex', alignItems: 'center', gap: '6px' }}
        >
            <span>{loading ? '⚙️...' : '⚙️'}</span> {loading ? 'Personalizing...' : 'Personalization'}
        </button>
    );
};

export const UrduTranslationButton = () => {
    const [isUrdu, setIsUrdu] = React.useState(false);
    const [loading, setLoading] = React.useState(false);
    const [originalContent, setOriginalContent] = React.useState('');

    const handleTranslate = async () => {
        let targetEl = document.getElementById('module-content');
        if (!targetEl) targetEl = document.querySelector('article');
        if (!targetEl) return;

        if (isUrdu) {
            // Restore English
            if (originalContent) targetEl.innerHTML = originalContent;
            setIsUrdu(false);
            return;
        }

        // Save original
        setOriginalContent(targetEl.innerHTML);
        setLoading(true);

        try {
            const textContent = targetEl.innerText;
            const res = await fetch('http://localhost:8000/api/translate', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    text: textContent,
                    target_lang: 'ur'
                })
            });
            const data = await res.json();
            if (!res.ok) throw new Error(data.detail || 'Translation failed');

            const translatedHtml = data.translated_text
                .replace(/\n/g, '<br/>');

            targetEl.innerHTML = `<div class="theme-doc-markdown markdown" dir="rtl" style="text-align: right; font-family: 'Noto Nastaliq Urdu', serif;">
                ${translatedHtml}
            </div>`;
            setIsUrdu(true);

        } catch (e: any) {
            alert("Error translating: " + e.message);
        } finally {
            setLoading(false);
        }
    };

    return (
        <button
            className="button button--primary button--sm icon-text"
            onClick={handleTranslate}
            disabled={loading}
            style={{ display: 'inline-flex', alignItems: 'center', gap: '6px' }}
        >
            <span>🌐</span> {loading ? 'Translating...' : (isUrdu ? 'English' : 'Urdu / اردو')}
        </button>
    );
};

export const ButtonContainer = ({ children }) => (
    <div style={{ marginBottom: '1.5rem', display: 'flex', gap: '0.5rem' }}>
        {children}
    </div>
);
