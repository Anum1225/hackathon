
import React, { useState, useRef, useEffect } from 'react';
import styles from './styles.module.css';
import { MessageSquare, X, Send, Bot, Sparkles, Trash2 } from 'lucide-react';

interface Message {
    id: string;
    text: string;
    sender: 'user' | 'bot';
    timestamp: Date;
    isHtml?: boolean; // Flag to render as HTML
}

export interface ChatbotRef {
    openChat: (msg?: string) => void;
}

const Chatbot = React.forwardRef<ChatbotRef>((props, ref) => {
    const [isOpen, setIsOpen] = useState(false);
    const [isClosing, setIsClosing] = useState(false); // For exit animation
    const [messages, setMessages] = useState<Message[]>([
        {
            id: '1',
            text: "Hello! I'm your Physical AI assistant. Ask me about ROS 2 or Robotics.",
            sender: 'bot',
            timestamp: new Date()
        }
    ]);
    const [inputText, setInputText] = useState('');
    const [isTyping, setIsTyping] = useState(false);
    const messagesEndRef = useRef<HTMLDivElement>(null);

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
        setMessages([{
            id: Date.now().toString(),
            text: "Chat cleared. Start a new topic!",
            sender: 'bot',
            timestamp: new Date()
        }]);
    };

    const scrollToBottom = () => {
        messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
    };

    useEffect(() => {
        if (isOpen) scrollToBottom();
    }, [messages, isTyping, isOpen]);

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
                    user_id: userId || undefined
                })
            });

            const data = await res.json();

            if (!res.ok) throw new Error(data.detail || 'Failed to get response');

            const botText = data.response;
            // The backend returns markdown. We might need a markdown parser. 
            // For now, simpler to just display text or simple HTML formatting if backend returns it.
            // Our backend returns text. 
            // Ideally we use a library like 'react-markdown' but for hackathon speed we'll just render text 
            // or simple replace \n with <br>.

            // Basic Markdown to HTML (Bold/List) - very rudimentary
            const formattedText = botText
                .replace(/\*\*(.*?)\*\*/g, '<b>$1</b>')
                .replace(/\n/g, '<br/>')
                .replace(/- (.*?)(<br\/>|$)/g, '<li>$1</li>');

            const botMsg: Message = {
                id: (Date.now() + 1).toString(),
                text: formattedText,
                sender: 'bot',
                timestamp: new Date(),
                isHtml: true,
                sources: data.sources // Assuming interface update if we want to show sources
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
                                className={`${styles.message} ${msg.sender === 'user' ? styles.userMessage : styles.botMessage}`}
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
