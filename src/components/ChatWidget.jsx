/**
 * ChatKit Widget - RAG Chatbot with Global and Selection RAG support
 */
import React, { useState, useRef, useEffect, useCallback } from 'react';
import clsx from 'clsx';
import styles from './ChatWidget.module.css';

const API_URL = process.env.DOCUSAURUS_CUSTOM_API_URL || 'http://localhost:8000/api/v1';

/**
 * Main ChatWidget component with RAG mode support
 */
function ChatWidget({ currentDoc = null }) {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    {
      role: 'assistant',
      content: "Hi! I'm your Physical AI learning assistant. Ask me anything about the textbook, or highlight text for contextual help!",
    },
  ]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [ragMode, setRagMode] = useState('global'); // 'global' or 'selection'
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef(null);

  // Get current document slug
  const getDocumentSlug = useCallback(() => {
    if (!currentDoc) {
      // Try to get from window location
      if (typeof window !== 'undefined') {
        const path = window.location.pathname;
        return path.replace(/\/docs\//, '').replace(/\//g, '_') || 'intro';
      }
    }
    return currentDoc?.slug || 'intro';
  }, [currentDoc]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages, isOpen]);

  // Handle text selection for Selection RAG
  useEffect(() => {
    const handleSelectionChange = () => {
      const selection = window.getSelection();
      if (selection && selection.toString().length > 0) {
        setSelectedText(selection.toString());
        setRagMode('selection');
      }
    };

    document.addEventListener('selectionchange', handleSelectionChange);
    return () => document.removeEventListener('selectionchange', handleSelectionChange);
  }, []);

  const handleSend = async () => {
    if (!input.trim() || isLoading) return;

    const userMessage = {
      role: 'user',
      content: input.trim(),
      mode: ragMode,
    };
    setMessages((prev) => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);

    try {
      const endpoint = ragMode === 'selection' && selectedText
        ? `${API_URL}/chat/selection`
        : `${API_URL}/chat`;

      const requestBody = {
        message: userMessage.content,
        history: messages,
        stream: false,
        mode: ragMode,
        document_slug: getDocumentSlug(),
        selected_text: selectedText || undefined,
      };

      const response = await fetch(endpoint, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      if (!response.ok) {
        throw new Error('Failed to send message');
      }

      const data = await response.json();

      setMessages((prev) => [
        ...prev,
        {
          role: 'assistant',
          content: data.answer,
          sources: data.sources,
          mode: data.mode,
        },
      ]);

      // Reset to global mode after selection response
      if (ragMode === 'selection') {
        setRagMode('global');
        setSelectedText('');
        window.getSelection().removeAllRanges();
      }
    } catch (error) {
      console.error('Chat error:', error);
      setMessages((prev) => [
        ...prev,
        {
          role: 'assistant',
          content: 'Sorry, I encountered an error. Please try again.',
        },
      ]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  const toggleRagMode = () => {
    setRagMode((prev) => (prev === 'global' ? 'selection' : 'global'));
    setSelectedText('');
  };

  return (
    <div className={styles.chatWidgetContainer}>
      {/* Chat Window */}
      <div
        className={clsx(styles.chatWindow, {
          [styles.chatWindowOpen]: isOpen,
        })}
      >
        <div className={styles.chatHeader}>
          <div className={styles.headerLeft}>
            <h3>Physical AI Assistant</h3>
            <span className={clsx(styles.ragBadge, styles[ragMode])}>
              {ragMode === 'selection' ? 'Selection RAG' : 'Global RAG'}
            </span>
          </div>
          <button
            className={styles.chatCloseBtn}
            onClick={() => setIsOpen(false)}
            aria-label="Close chat"
          >
            &times;
          </button>
        </div>

        {/* RAG Mode Toggle */}
        <div className={styles.ragToggle}>
          <button
            className={clsx(styles.ragToggleBtn, {
              [styles.active]: ragMode === 'global',
            })}
            onClick={() => setRagMode('global')}
            title="Search entire textbook"
          >
            Global Search
          </button>
          <button
            className={clsx(styles.ragToggleBtn, {
              [styles.active]: ragMode === 'selection',
              [styles.hasSelection]: selectedText,
            })}
            onClick={toggleRagMode}
            title={selectedText ? `Selected: ${selectedText.substring(0, 30)}...` : 'Select text first'}
          >
            {ragMode === 'selection' ? 'Selection Mode' : 'Context Help'}
          </button>
        </div>

        {/* Selected text indicator */}
        {selectedText && (
          <div className={styles.selectedIndicator}>
            <span>Selected: "{selectedText.substring(0, 50)}{selectedText.length > 50 ? '...' : ''}"</span>
          </div>
        )}

        <div className={styles.chatMessages}>
          {messages.map((msg, idx) => (
            <div
              key={idx}
              className={clsx(styles.chatMessage, styles[msg.role], styles[msg.mode])}
            >
              <div className={styles.messageContent}>{msg.content}</div>
              {msg.sources && msg.sources.length > 0 && (
                <div className={styles.sources}>
                  <span className={styles.sourcesLabel}>Sources:</span>
                  {msg.sources.map((source, sidx) => (
                    <span key={sidx} className={styles.sourceTag}>
                      {source.source || `Doc ${sidx + 1}`}
                    </span>
                  ))}
                </div>
              )}
            </div>
          ))}
          {isLoading && (
            <div className={clsx(styles.chatMessage, styles.assistant)}>
              <div className={styles.loadingDots}>
                <span></span>
                <span></span>
                <span></span>
              </div>
            </div>
          )}
          <div ref={messagesEndRef} />
        </div>

        <div className={styles.chatInputContainer}>
          <input
            type="text"
            className={styles.chatInput}
            placeholder={ragMode === 'selection' && selectedText
              ? "Ask about the selected text..."
              : "Ask a question..."}
            value={input}
            onChange={(e) => setInput(e.target.value)}
            onKeyPress={handleKeyPress}
            disabled={isLoading}
          />
          <button
            className={styles.chatSendBtn}
            onClick={handleSend}
            disabled={isLoading || !input.trim()}
            aria-label="Send message"
          >
            <svg
              width="20"
              height="20"
              viewBox="0 0 24 24"
              fill="none"
              xmlns="http://www.w3.org/2000/svg"
            >
              <path
                d="M2.01 21L23 12L2.01 3L2 10L17 12L2 14L2.01 21Z"
                fill="currentColor"
              />
            </svg>
          </button>
        </div>
      </div>

      {/* Toggle Button */}
      <button
        className={styles.chatToggleButton}
        onClick={() => setIsOpen(!isOpen)}
        aria-label={isOpen ? 'Close chat' : 'Open chat'}
      >
        {isOpen ? (
          <svg
            width="24"
            height="24"
            viewBox="0 0 24 24"
            fill="none"
            xmlns="http://www.w3.org/2000/svg"
          >
            <path
              d="M18 6L6 18M6 6L18 18"
              stroke="currentColor"
              strokeWidth="2"
              strokeLinecap="round"
              strokeLinejoin="round"
            />
          </svg>
        ) : (
          <svg
            width="24"
            height="24"
            viewBox="0 0 24 24"
            fill="none"
            xmlns="http://www.w3.org/2000/svg"
          >
            <path
              d="M21 15C21 15.5304 20.7893 16.0391 20.4142 16.4142C20.0391 16.7893 19.5304 17 19 17H7L3 21V5C3 4.46957 3.21071 3.96086 3.58579 3.58579C3.96086 3.21071 4.46957 3 5 3H19C19.5304 3 20.0391 3.21071 20.4142 3.58579C20.7893 3.96086 21 4.46957 21 5V15Z"
              stroke="currentColor"
              strokeWidth="2"
              strokeLinecap="round"
              strokeLinejoin="round"
            />
          </svg>
        )}
      </button>
    </div>
  );
}

export default ChatWidget;
