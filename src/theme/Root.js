
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
            // IMPORTANT: Only intercept if the href is EXACTLY /auth/login
            // If we've changed it to /profile, this check MUST fail.
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

        // Navbar Login -> Profile Swap Logic
        const updateNavbar = () => {
            const userName = localStorage.getItem('user_name');
            const loginLink = document.querySelector('a[href="/auth/login"]');

            if (userName && loginLink) {
                // Check if already updated to avoid infinite loop of mutations if we were changing attributes that trigger it
                // But since we change href, the querySelector won't match anymore!

                loginLink.innerHTML = `<svg xmlns="http://www.w3.org/2000/svg" width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" class="lucide lucide-user" style="margin-right: 6px; vertical-align: text-bottom;"><path d="M19 21v-2a4 4 0 0 0-4-4H9a4 4 0 0 0-4 4v2"/><circle cx="12" cy="7" r="4"/></svg> ${userName}`;
                loginLink.setAttribute('href', '/profile');
            }
        };

        // Mutation Observer to handle Docusaurus re-renders/hydration
        const observer = new MutationObserver(() => {
            updateNavbar();
        });

        // Start observing the body for changes
        observer.observe(document.body, { childList: true, subtree: true });

        // Initial check
        updateNavbar();

        // Add Capture phase listener for maximum priority on links
        document.addEventListener('click', handleLinkClick, true);
        document.addEventListener('mouseup', handleSelection);
        document.addEventListener('scroll', () => setSelection(null));

        return () => {
            observer.disconnect();
            document.removeEventListener('click', handleLinkClick, true);
            document.removeEventListener('mouseup', handleSelection);
        };
    }, []); // Run once on mount, let Observer handle the rest

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
                        position: 'fixed', // Changed from absolute to fixed to prevent scrolling issues
                        top: selection.y - 40, // Adjusted positioning
                        left: selection.x,
                        transform: 'translate(-50%, 0)',
                        zIndex: 999999, // High Z-Index
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
                            from { opacity: 0; transform: translate(-50%, 20px) scale(0.8); }
                            to { opacity: 1; transform: translate(-50%, 0) scale(1); }
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
