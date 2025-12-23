/**
 * Zustand store for model state management
 */

import { create } from 'zustand';
import type {
  ModelState,
  Node,
  Beam,
  Material,
  Section,
  ChatMessage,
  AnalysisResults,
  ModelEvent,
} from '../types/model';
import api from '../api/client';
import { sseClient } from '../api/events';

interface UIState {
  leftPanelCollapsed: boolean;
  rightPanelCollapsed: boolean;
  activeRightTab: 'results' | 'chat';
  viewMode: 'fem' | 'results' | 'realistic';
  selectedBeamId: number | null;
  isLoading: boolean;
  error: string | null;
}

interface Store extends ModelState, UIState {
  // Chat state
  chatMessages: ChatMessage[];
  isChatProcessing: boolean;

  // UI actions
  toggleLeftPanel: () => void;
  toggleRightPanel: () => void;
  setActiveRightTab: (tab: 'results' | 'chat') => void;
  setViewMode: (mode: 'fem' | 'results' | 'realistic') => void;
  selectBeam: (beamId: number | null) => void;
  setError: (error: string | null) => void;

  // Model actions
  fetchModelState: () => Promise<void>;
  resetModel: () => Promise<void>;
  updateFromEvent: (event: ModelEvent) => void;

  // Chat actions
  sendChatMessage: (content: string) => Promise<void>;
  clearChat: () => void;

  // SSE
  initializeSSE: () => void;
  disconnectSSE: () => void;
}

export const useStore = create<Store>((set, get) => ({
  // Initial model state
  name: 'New Model',
  nodes: [],
  beams: [],
  materials: [],
  sections: [],
  boundaryConditions: [],
  loadCases: [],
  results: null,
  isAnalyzed: false,

  // Initial UI state
  leftPanelCollapsed: false,
  rightPanelCollapsed: false,
  activeRightTab: 'chat',
  viewMode: 'fem',
  selectedBeamId: null,
  isLoading: false,
  error: null,

  // Chat state
  chatMessages: [],
  isChatProcessing: false,

  // UI actions
  toggleLeftPanel: () =>
    set((state) => ({ leftPanelCollapsed: !state.leftPanelCollapsed })),

  toggleRightPanel: () =>
    set((state) => ({ rightPanelCollapsed: !state.rightPanelCollapsed })),

  setActiveRightTab: (tab) => set({ activeRightTab: tab }),

  setViewMode: (mode) => set({ viewMode: mode }),

  selectBeam: (beamId) => set({ selectedBeamId: beamId }),

  setError: (error) => set({ error }),

  // Model actions
  fetchModelState: async () => {
    set({ isLoading: true, error: null });
    const response = await api.model.getState();

    if (response.success && response.data) {
      set({
        name: response.data.name,
        nodes: response.data.nodes as Node[],
        beams: response.data.beams as Beam[],
        materials: response.data.materials as Material[],
        sections: response.data.sections as Section[],
        isLoading: false,
      });
    } else {
      set({ isLoading: false, error: response.error || 'Failed to fetch model state' });
    }
  },

  resetModel: async () => {
    set({ isLoading: true, error: null });
    const response = await api.model.reset();

    if (response.success) {
      set({
        name: 'New Model',
        nodes: [],
        beams: [],
        materials: [],
        sections: [],
        boundaryConditions: [],
        loadCases: [],
        results: null,
        isAnalyzed: false,
        isLoading: false,
      });
    } else {
      set({ isLoading: false, error: response.error || 'Failed to reset model' });
    }
  },

  updateFromEvent: (event: ModelEvent) => {
    switch (event.event_type) {
      case 'model_updated':
        get().fetchModelState();
        break;
      case 'analysis_complete':
        set({
          isAnalyzed: true,
          results: event.data.results as AnalysisResults,
        });
        break;
      case 'error':
        set({ error: event.data.message as string });
        break;
      case 'tool_executed':
        // Refresh model state after tool execution
        get().fetchModelState();
        break;
    }
  },

  // Chat actions
  sendChatMessage: async (content: string) => {
    const userMessage: ChatMessage = {
      id: crypto.randomUUID(),
      role: 'user',
      content,
      timestamp: new Date(),
    };

    set((state) => ({
      chatMessages: [...state.chatMessages, userMessage],
      isChatProcessing: true,
    }));

    // Build history from previous messages
    const history = get().chatMessages.map((msg) => ({
      role: msg.role,
      content: msg.content,
    }));

    const response = await api.chat.send(content, history);

    if (response.success && response.data) {
      const assistantMessage: ChatMessage = {
        id: crypto.randomUUID(),
        role: 'assistant',
        content: response.data.response,
        timestamp: new Date(),
        toolCalls: response.data.tool_calls,
      };

      set((state) => ({
        chatMessages: [...state.chatMessages, assistantMessage],
        isChatProcessing: false,
      }));

      // Refresh model state if tools modified the model
      if (response.data.model_updated) {
        get().fetchModelState();
      }
    } else {
      const errorMessage: ChatMessage = {
        id: crypto.randomUUID(),
        role: 'assistant',
        content: `Error: ${response.error || 'Failed to get response'}`,
        timestamp: new Date(),
      };

      set((state) => ({
        chatMessages: [...state.chatMessages, errorMessage],
        isChatProcessing: false,
      }));
    }
  },

  clearChat: () => set({ chatMessages: [] }),

  // SSE
  initializeSSE: () => {
    sseClient.connect();
    sseClient.subscribe((event) => {
      get().updateFromEvent(event);
    });
  },

  disconnectSSE: () => {
    sseClient.disconnect();
  },
}));

export default useStore;
