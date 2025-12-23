/**
 * Server-Sent Events client for real-time model updates
 */

import type { ModelEvent } from '../types/model';

type EventHandler = (event: ModelEvent) => void;

class SSEClient {
  private eventSource: EventSource | null = null;
  private handlers: Set<EventHandler> = new Set();
  private reconnectAttempts = 0;
  private maxReconnectAttempts = 5;
  private reconnectDelay = 1000;

  connect(): void {
    if (this.eventSource) {
      return;
    }

    try {
      this.eventSource = new EventSource('/api/events/stream');

      this.eventSource.onopen = () => {
        console.log('SSE connection established');
        this.reconnectAttempts = 0;
      };

      this.eventSource.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data) as ModelEvent;
          this.notifyHandlers(data);
        } catch (error) {
          console.error('Failed to parse SSE event:', error);
        }
      };

      this.eventSource.onerror = (error) => {
        console.error('SSE connection error:', error);
        this.handleReconnect();
      };
    } catch (error) {
      console.error('Failed to create EventSource:', error);
      this.handleReconnect();
    }
  }

  private handleReconnect(): void {
    this.disconnect();

    if (this.reconnectAttempts < this.maxReconnectAttempts) {
      this.reconnectAttempts++;
      const delay = this.reconnectDelay * Math.pow(2, this.reconnectAttempts - 1);
      console.log(`Reconnecting in ${delay}ms (attempt ${this.reconnectAttempts})`);
      setTimeout(() => this.connect(), delay);
    } else {
      console.error('Max reconnect attempts reached');
    }
  }

  disconnect(): void {
    if (this.eventSource) {
      this.eventSource.close();
      this.eventSource = null;
    }
  }

  subscribe(handler: EventHandler): () => void {
    this.handlers.add(handler);
    return () => this.handlers.delete(handler);
  }

  private notifyHandlers(event: ModelEvent): void {
    this.handlers.forEach((handler) => {
      try {
        handler(event);
      } catch (error) {
        console.error('Error in SSE handler:', error);
      }
    });
  }

  isConnected(): boolean {
    return this.eventSource?.readyState === EventSource.OPEN;
  }
}

// Singleton instance
export const sseClient = new SSEClient();

export default sseClient;
