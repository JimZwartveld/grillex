import { ChatMessage as ChatMessageType } from '../../types/model';
import { Check, X, Wrench } from 'lucide-react';

interface Props {
  message: ChatMessageType;
}

export default function ChatMessage({ message }: Props) {
  const isUser = message.role === 'user';

  return (
    <div className={`flex ${isUser ? 'justify-end' : 'justify-start'}`}>
      <div
        className={`max-w-[85%] rounded-lg p-3 ${
          isUser
            ? 'bg-blue-600 text-white'
            : 'bg-gray-100 text-gray-800'
        }`}
      >
        {/* Message content */}
        <p className="text-sm whitespace-pre-wrap break-words">{message.content}</p>

        {/* Tool calls (for assistant messages) */}
        {message.toolCalls && message.toolCalls.length > 0 && (
          <div className="mt-2 pt-2 border-t border-gray-200">
            <p className="text-xs text-gray-500 mb-1 flex items-center gap-1">
              <Wrench className="w-3 h-3" />
              Actions performed:
            </p>
            <div className="space-y-1">
              {message.toolCalls.map((tc, i) => (
                <div
                  key={i}
                  className="text-xs bg-white bg-opacity-50 rounded px-2 py-1 flex items-center gap-1"
                >
                  <span className="font-mono text-gray-700">{tc.name}</span>
                  {tc.result !== undefined ? (
                    <Check className="w-3 h-3 text-green-600" />
                  ) : (
                    <X className="w-3 h-3 text-red-500" />
                  )}
                </div>
              ))}
            </div>
          </div>
        )}

        {/* Timestamp */}
        <p
          className={`text-xs mt-1 ${
            isUser ? 'text-blue-200' : 'text-gray-400'
          }`}
        >
          {message.timestamp.toLocaleTimeString([], {
            hour: '2-digit',
            minute: '2-digit',
          })}
        </p>
      </div>
    </div>
  );
}
