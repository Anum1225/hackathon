
import React, { useState, useRef, useEffect } from 'react';
import styles from './styles.module.css';
import { MessageSquare, X, Send, Bot, Sparkles, Trash2, Languages, Smile } from 'lucide-react';

interface Message {
    id: string;
    text: string;
    sender: 'user' | 'bot';
    timestamp: Date;
    isHtml?: boolean;
    isUrdu?: boolean;
}

export interface ChatbotRef {
    openChat: (msg?: string) => void;
}

const Chatbot = React.forwardRef<ChatbotRef>((props, ref) => {
    const [isOpen, setIsOpen] = useState(false);
    const [isClosing, setIsClosing] = useState(false);
    const [inputText, setInputText] = useState('');
    const [isTyping, setIsTyping] = useState(false);
    const [isUrdu, setIsUrdu] = useState(false);
    const [isPersonalized, setIsPersonalized] = useState(false);
    const messagesEndRef = useRef<HTMLDivElement>(null);
    
    const userId = localStorage.getItem('user_id') || 'guest';
    const storageKey = `chatHistory_${userId}`;
    
    const [messages, setMessages] = useState<Message[]>(() => {
        const saved = localStorage.getItem(storageKey);
        if (saved) {
            try {
                const parsed = JSON.parse(saved);
                return parsed.map(m => ({ ...m, timestamp: new Date(m.timestamp) }));
            } catch { }
        }
        return [{
            id: '1',
            text: "Hello! I'm your Physical AI assistant. Ask me about ROS 2 or Robotics.",
            sender: 'bot',
            timestamp: new Date()
        }];
    });

    const startMessages = [
        "What is Physical AI?",
        "Explain ROS 2 nodes",
        "How to set up Isaac Sim?"
    ];

    React.useImperativeHandle(ref, () => ({
        openChat: (msg?: string) => {
            setIsOpen(true);
            setIsClosing(false);
            if (msg) {
                setTimeout(() => handleSend(msg), 150);
            }
        }
    }));

    // Handle Close Animation
    const handleClose = () => {
        setIsClosing(true);
        setTimeout(() => {
            setIsOpen(false);
            setIsClosing(false);
        }, 300); // Match CSS animation duration
    };

    const handleClear = () => {
        const newMsgs = [{
            id: Date.now().toString(),
            text: "Chat cleared. Start a new topic!",
            sender: 'bot',
            timestamp: new Date()
        }];
        setMessages(newMsgs);
        localStorage.setItem(storageKey, JSON.stringify(newMsgs));
    };

    const scrollToBottom = () => {
        messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
    };

    useEffect(() => {
        if (isOpen) scrollToBottom();
    }, [messages, isTyping, isOpen]);
    
    useEffect(() => {
        localStorage.setItem(storageKey, JSON.stringify(messages));
    }, [messages, storageKey]);

    const handleSend = async (text: string) => {
        if (!text.trim()) return;

        const userMsg: Message = {
            id: Date.now().toString(),
            text: text,
            sender: 'user',
            timestamp: new Date()
        };

        const newMessages = [...messages, userMsg];
        setMessages(newMessages);
        setInputText('');
        setIsTyping(true);

        try {
            // Prepare history for API
            const history = messages
                .filter(m => m.id !== '1') // Skip welcome
                .map(m => ({
                    role: m.sender,
                    content: m.text
                }));

            const userId = localStorage.getItem('user_id');

            const res = await fetch('http://localhost:8000/api/chat', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    message: text,
                    history: history,
                    user_id: userId || undefined,
                    translate_to_urdu: isUrdu,
                    personalize: isPersonalized
                })
            });

            const data = await res.json();

            if (!res.ok) throw new Error(data.detail || 'Failed to get response');

            let botText = data.response;
            
            // Enhanced formatting
            let formattedText = botText
                .replace(/\*\*\*(.+?)\*\*\*/g, '<b><i>$1</i></b>')
                .replace(/\*\*(.+?)\*\*/g, '<b>$1</b>')
                .replace(/\*(.+?)\*/g, '<i>$1</i>')
                .replace(/`(.+?)`/g, '<code>$1</code>');
            
            // Tables
            formattedText = formattedText.replace(/\|(.+?)\|/g, (match) => {
                const cells = match.split('|').filter(c => c.trim());
                return '<tr>' + cells.map(c => `<td>${c.trim()}</td>`).join('') + '</tr>';
            });
            if (formattedText.includes('<tr>')) {
                formattedText = '<table>' + formattedText + '</table>';
            }
            
            // Lists with emojis for personalized mode
            if (isPersonalized) {
                formattedText = formattedText
                    .replace(/^(\d+)\. (.+)$/gm, '✨ <b>$1.</b> $2')
                    .replace(/^- (.+)$/gm, '🔹 $1')
                    .replace(/^\* (.+)$/gm, '⭐ $1');
            } else {
                formattedText = formattedText
                    .replace(/^(\d+)\. (.+)$/gm, '<li>$2</li>')
                    .replace(/^- (.+)$/gm, '<li>$1</li>')
                    .replace(/^\* (.+)$/gm, '<li>$1</li>');
                if (formattedText.includes('<li>')) {
                    formattedText = '<ul>' + formattedText + '</ul>';
                }
            }
            
            formattedText = formattedText.replace(/\n/g, '<br/>');

            const botMsg: Message = {
                id: (Date.now() + 1).toString(),
                text: formattedText,
                sender: 'bot',
                timestamp: new Date(),
                isHtml: true,
                isUrdu: isUrdu
            };

            setMessages(prev => [...prev, botMsg]);

        } catch (error) {
            console.error(error);
            const errorMsg: Message = {
                id: Date.now().toString(),
                text: "Sorry, I'm having trouble connecting to the brain. Please try again.",
                sender: 'bot',
                timestamp: new Date()
            };
            setMessages(prev => [...prev, errorMsg]);
        } finally {
            setIsTyping(false);
        }
    };

    const handleKeyPress = (e: React.KeyboardEvent) => {
        if (e.key === 'Enter') handleSend(inputText);
    };

    // Render Logic
    if (!isOpen && !isClosing && !messages.length) return null; // Keep button visible though

    return (
        <div className={styles.container}>
            {(!isOpen && !isClosing) && (
                <button
                    className={styles.toggleButton}
                    onClick={() => setIsOpen(true)}
                    aria-label="Open Chat"
                >
                    <MessageSquare size={26} />
                </button>
            )}

            {(isOpen || isClosing) && (
                <div className={`${styles.chatWindow} ${isClosing ? styles.animateClose : styles.animateOpen}`}>
                    {/* Header */}
                    <div className={styles.header}>
                        <div className={styles.title}>
                            <Bot size={18} />
                            <span>AI Tutor</span>
                            <div className={styles.onlineBadge} />
                        </div>
                        <div className={styles.headerActions}>
                            <button 
                                className={`${styles.iconBtn} ${isUrdu ? styles.active : ''}`} 
                                onClick={() => setIsUrdu(!isUrdu)} 
                                title="Toggle Urdu"
                            >
                                <Languages size={16} />
                            </button>
                            <button 
                                className={`${styles.iconBtn} ${isPersonalized ? styles.active : ''}`} 
                                onClick={() => setIsPersonalized(!isPersonalized)} 
                                title="Personalize"
                            >
                                <Smile size={16} />
                            </button>
                            <button className={styles.iconBtn} onClick={handleClear} title="Clear Chat">
                                <Trash2 size={16} />
                            </button>
                            <button className={styles.iconBtn} onClick={handleClose} title="Close">
                                <X size={18} />
                            </button>
                        </div>
                    </div>

                    {/* Messages */}
                    <div className={styles.messages}>
                        {messages.map((msg) => (
                            <div
                                key={msg.id}
                                className={`${styles.message} ${msg.sender === 'user' ? styles.userMessage : styles.botMessage} ${msg.isUrdu ? styles.urduMessage : ''}`}
                                dir={msg.isUrdu ? 'rtl' : 'ltr'}
                            >
                                {msg.isHtml ? (
                                    <span dangerouslySetInnerHTML={{ __html: msg.text }} />
                                ) : (
                                    msg.text
                                )}
                            </div>
                        ))}
                        {isTyping && (
                            <div className={styles.typing}>
                                <Sparkles size={12} style={{ display: 'inline', marginRight: '4px' }} />
                                Thinking...
                            </div>
                        )}

                        {/* Suggestions */}
                        {messages.length < 2 && !isTyping && (
                            <div style={{ display: 'flex', flexWrap: 'wrap', gap: '6px', marginTop: 'auto' }}>
                                {startMessages.map((msg, i) => (
                                    <button
                                        key={i}
                                        style={{
                                            background: 'rgba(139, 92, 246, 0.1)', border: '1px solid rgba(139, 92, 246, 0.3)',
                                            borderRadius: '12px', padding: '6px 12px', color: '#cbd5e1', fontSize: '0.75rem',
                                            cursor: 'pointer'
                                        }}
                                        onClick={() => handleSend(msg)}
                                    >
                                        {msg}
                                    </button>
                                ))}
                            </div>
                        )}

                        <div ref={messagesEndRef} />
                    </div>

                    {/* Input */}
                    <div className={styles.inputArea}>
                        <input
                            className={styles.input}
                            placeholder="Ask anything..."
                            value={inputText}
                            onChange={(e) => setInputText(e.target.value)}
                            onKeyPress={handleKeyPress}
                        />
                        <button
                            className={styles.sendButton}
                            onClick={() => handleSend(inputText)}
                            disabled={!inputText.trim()}
                        >
                            <Send size={16} />
                        </button>
                    </div>
                </div>
            )}
        </div>
    );
});

export default Chatbot;
