
import React, { useState, useEffect, useRef } from 'react';
import Chatbot from '@site/src/components/Chatbot';
import AuthModal from '@site/src/components/AuthModal';
import { Sparkles } from 'lucide-react';
import { useLocation } from '@docusaurus/router'; // Docusaurus router hook

export default function Root({ children }) {
    const [isAuthOpen, setIsAuthOpen] = useState(false);
    const [selection, setSelection] = useState(null);
    const chatbotRef = useRef(null);
    const location = useLocation(); // Get current route

    useEffect(() => {
        // Robust Intercept for Login Link
        const handleLinkClick = (e) => {
            // Find closest anchor tag
            const target = e.target.closest('a');

            // Check if it's the login link
            if (target && target.getAttribute('href') === '/auth/login') {
                e.preventDefault(); // STOP navigation
                e.stopPropagation(); // STOP other handlers (dropdowns etc)
                setIsAuthOpen(true); // Open Modal
                return false;
            }
        };

        // Text Selection Listener
        const handleSelection = () => {
            // Don't show on Homepage
            if (location.pathname === '/') {
                setSelection(null);
                return;
            }

            const selectedText = window.getSelection().toString().trim();

            if (selectedText.length > 5 && selectedText.length < 500) {
                const range = window.getSelection().getRangeAt(0);
                const rect = range.getBoundingClientRect();

                setSelection({
                    text: selectedText,
                    x: rect.left + (rect.width / 2),
                    y: rect.top + window.scrollY - 10
                });
            } else {
                setSelection(null);
            }
        };

        const handleClear = (e) => {
            if (e.target.closest('#ask-ai-btn')) return;
            if (!window.getSelection().toString().trim()) {
                setSelection(null);
            }
        };

        // Force cleanup selection on route change
        setSelection(null);

        // Add Capture phase listener for maximum priority on links
        document.addEventListener('click', handleLinkClick, true);
        document.addEventListener('mouseup', handleSelection);
        document.addEventListener('scroll', () => setSelection(null));

        return () => {
            document.removeEventListener('click', handleLinkClick, true);
            document.removeEventListener('mouseup', handleSelection);
        };
    }, [location.pathname]); // Re-run if route changes (though mainly for cleanup)

    const handleAskAI = (e) => {
        e.stopPropagation();
        if (selection && chatbotRef.current) {
            chatbotRef.current.openChat(`Explain this: ${selection.text}`);
            setSelection(null);
            window.getSelection().removeAllRanges();
        }
    };

    // Helper: Is Homepage?
    const isHomePage = location.pathname === '/';

    return (
        <>
            {children}

            {/* Ask AI Button - Only if NOT homepage & selection data exists */}
            {!isHomePage && selection && (
                <button
                    id="ask-ai-btn"
                    onClick={handleAskAI}
                    style={{
                        position: 'absolute',
                        top: selection.y,
                        left: selection.x,
                        transform: 'translate(-50%, -100%)',
                        zIndex: 2147483648,
                        background: 'linear-gradient(135deg, #7c3aed 0%, #06b6d4 100%)',
                        border: 'none',
                        borderRadius: '20px',
                        padding: '8px 16px',
                        color: 'white',
                        fontWeight: '600',
                        fontSize: '0.85rem',
                        cursor: 'pointer',
                        boxShadow: '0 4px 15px rgba(124, 58, 237, 0.4)',
                        display: 'flex',
                        alignItems: 'center',
                        gap: '6px',
                        animation: 'popIn 0.2s cubic-bezier(0.175, 0.885, 0.32, 1.275)'
                    }}
                >
                    <Sparkles size={14} />
                    Ask AI
                    <style>{`
                        @keyframes popIn {
                            from { opacity: 0; transform: translate(-50%, -80%) scale(0.8); }
                            to { opacity: 1; transform: translate(-50%, -100%) scale(1); }
                        }
                    `}</style>
                </button>
            )}

            <Chatbot ref={chatbotRef} />
            <AuthModal
                isOpen={isAuthOpen}
                onClose={() => setIsAuthOpen(false)}
            />
        </>
    );
}
