/**
 * API client for Grillex backend
 */

const API_BASE = '/api';

export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: string;
}

async function request<T>(
  endpoint: string,
  options: RequestInit = {}
): Promise<ApiResponse<T>> {
  try {
    const response = await fetch(`${API_BASE}${endpoint}`, {
      headers: {
        'Content-Type': 'application/json',
        ...options.headers,
      },
      ...options,
    });

    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      return {
        success: false,
        error: errorData.detail || `HTTP ${response.status}: ${response.statusText}`,
      };
    }

    const data = await response.json();
    return { success: true, data };
  } catch (error) {
    return {
      success: false,
      error: error instanceof Error ? error.message : 'Unknown error',
    };
  }
}

// Model endpoints
export const modelApi = {
  getState: () => request<{
    name: string;
    nodes: unknown[];
    beams: unknown[];
    materials: unknown[];
    sections: unknown[];
  }>('/model/state'),

  reset: () => request<{ message: string }>('/model/reset', { method: 'POST' }),
};

// Tool endpoints
export const toolsApi = {
  list: () => request<{ tools: unknown[] }>('/tools'),

  execute: (name: string, params: Record<string, unknown>) =>
    request<{ success: boolean; result?: unknown; error?: string }>('/tools/execute', {
      method: 'POST',
      body: JSON.stringify({ name, params }),
    }),
};

// Chat endpoints
export const chatApi = {
  send: async (message: string, history: Array<{ role: string; content: string }> = []) => {
    return request<{
      response: string;
      tool_calls: Array<{
        name: string;
        input: Record<string, unknown>;
        result: unknown;
      }>;
      model_updated: boolean;
    }>('/chat', {
      method: 'POST',
      body: JSON.stringify({ message, history }),
    });
  },

  getStatus: () => request<{ is_processing: boolean; current_tool?: string }>('/chat/status'),
};

export default {
  model: modelApi,
  tools: toolsApi,
  chat: chatApi,
};
