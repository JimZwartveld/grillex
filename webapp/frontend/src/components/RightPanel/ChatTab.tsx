import { useState, useRef, useEffect } from 'react';
import { Send, Loader2 } from 'lucide-react';
import { Button } from '../common';
import ChatMessage from './ChatMessage';
import useStore from '../../stores/modelStore';

export default function ChatTab() {
  const { chatMessages, isChatProcessing, sendChatMessage, clearChat } = useStore();
  const [input, setInput] = useState('');
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLTextAreaElement>(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [chatMessages]);

  const handleSend = async () => {
    const trimmedInput = input.trim();
    if (!trimmedInput || isChatProcessing) return;

    setInput('');
    await sendChatMessage(trimmedInput);
    inputRef.current?.focus();
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  return (
    <div className="h-full flex flex-col">
      {/* Header with clear button */}
      <div className="flex items-center justify-between px-4 py-2 border-b border-gray-100">
        <h3 className="text-sm font-medium text-gray-600">AI Assistant</h3>
        {chatMessages.length > 0 && (
          <button
            onClick={clearChat}
            className="text-xs text-gray-400 hover:text-gray-600"
          >
            Clear chat
          </button>
        )}
      </div>

      {/* Messages area */}
      <div className="flex-1 overflow-y-auto p-4 space-y-3">
        {chatMessages.length === 0 ? (
          <div className="text-center text-gray-400 mt-8">
            <p className="text-sm mb-2">Welcome to the AI Assistant!</p>
            <p className="text-xs">
              Try asking me to:
            </p>
            <ul className="text-xs mt-2 space-y-1">
              <li>• "Create a 6m cantilever beam"</li>
              <li>• "Add a 10 kN downward load at the tip"</li>
              <li>• "Run the analysis"</li>
              <li>• "What's the maximum deflection?"</li>
            </ul>
          </div>
        ) : (
          chatMessages.map((msg) => (
            <ChatMessage key={msg.id} message={msg} />
          ))
        )}

        {/* Loading indicator */}
        {isChatProcessing && (
          <div className="flex justify-start">
            <div className="bg-gray-100 rounded-lg p-3 flex items-center gap-2">
              <Loader2 className="w-4 h-4 text-gray-500 animate-spin" />
              <span className="text-sm text-gray-500">Thinking...</span>
            </div>
          </div>
        )}

        <div ref={messagesEndRef} />
      </div>

      {/* Input area */}
      <div className="p-3 border-t border-gray-100">
        <div className="flex gap-2">
          <textarea
            ref={inputRef}
            className="flex-1 border border-gray-200 rounded-lg p-2 text-sm resize-none focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent"
            rows={2}
            value={input}
            onChange={(e) => setInput(e.target.value)}
            onKeyDown={handleKeyDown}
            placeholder="Ask about your model..."
            disabled={isChatProcessing}
          />
          <Button
            onClick={handleSend}
            variant="primary"
            disabled={!input.trim() || isChatProcessing}
            className="self-end"
          >
            <Send className="w-4 h-4" />
          </Button>
        </div>
        <p className="text-xs text-gray-400 mt-1">
          Press Enter to send, Shift+Enter for new line
        </p>
      </div>
    </div>
  );
}
