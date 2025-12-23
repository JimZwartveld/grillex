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
  // Results invalidation warning
  resultWarning: string | null;
  // Context menu state
  contextMenu: {
    isOpen: boolean;
    x: number;
    y: number;
    elementType: 'beam' | 'support' | 'load' | 'cargo' | null;
    elementId: number | null;
  };
  // Add element context menu state (right-click on empty space)
  addContextMenu: {
    isOpen: boolean;
    x: number;
    y: number;
    worldPosition: [number, number, number] | null;
  };
  // Properties dialog state
  propertiesDialog: {
    isOpen: boolean;
    elementType: 'beam' | 'support' | 'load' | 'cargo' | null;
    elementId: number | null;
  };
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
  // Context menu actions
  openContextMenu: (x: number, y: number, elementType: 'beam' | 'support' | 'load' | 'cargo', elementId: number | null) => void;
  closeContextMenu: () => void;
  // Add context menu actions (right-click on empty space)
  openAddContextMenu: (x: number, y: number, worldPosition: [number, number, number]) => void;
  closeAddContextMenu: () => void;
  // Properties dialog actions
  openPropertiesDialog: (elementType: 'beam' | 'support' | 'load' | 'cargo', elementId: number | null) => void;
  closePropertiesDialog: () => void;
  // Results invalidation actions
  invalidateResults: () => void;
  clearResultWarning: () => void;

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
  cargos: [],
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
  resultWarning: null,
  contextMenu: {
    isOpen: false,
    x: 0,
    y: 0,
    elementType: null,
    elementId: null,
  },
  addContextMenu: {
    isOpen: false,
    x: 0,
    y: 0,
    worldPosition: null,
  },
  propertiesDialog: {
    isOpen: false,
    elementType: null,
    elementId: null,
  },

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

  // Context menu actions
  openContextMenu: (x, y, elementType, elementId) =>
    set({
      contextMenu: { isOpen: true, x, y, elementType, elementId },
    }),

  closeContextMenu: () =>
    set({
      contextMenu: { isOpen: false, x: 0, y: 0, elementType: null, elementId: null },
    }),

  // Add context menu actions
  openAddContextMenu: (x, y, worldPosition) =>
    set({
      addContextMenu: { isOpen: true, x, y, worldPosition },
      contextMenu: { isOpen: false, x: 0, y: 0, elementType: null, elementId: null }, // Close element menu
    }),

  closeAddContextMenu: () =>
    set({
      addContextMenu: { isOpen: false, x: 0, y: 0, worldPosition: null },
    }),

  // Properties dialog actions
  openPropertiesDialog: (elementType, elementId) =>
    set({
      propertiesDialog: { isOpen: true, elementType, elementId },
    }),

  closePropertiesDialog: () =>
    set({
      propertiesDialog: { isOpen: false, elementType: null, elementId: null },
    }),

  // Results invalidation actions
  invalidateResults: () =>
    set({
      isAnalyzed: false,
      results: null,
      resultWarning: 'Model changed. Re-run analysis for updated results.',
    }),

  clearResultWarning: () =>
    set({ resultWarning: null }),

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
        cargos: [],
        results: null,
        isAnalyzed: false,
        isLoading: false,
      });
    } else {
      set({ isLoading: false, error: response.error || 'Failed to reset model' });
    }
  },

  updateFromEvent: (event: ModelEvent) => {
    // Events that modify the model and should invalidate results
    const modelChangingEvents = [
      'beam_added', 'beam_updated', 'beam_deleted',
      'material_added', 'material_updated', 'material_deleted',
      'section_added', 'section_updated', 'section_deleted',
      'bc_added', 'bc_deleted',
      'load_added', 'load_deleted',
      'cargo_added', 'cargo_updated', 'cargo_deleted',
      'spring_added', 'spring_deleted',
    ];

    // Check if this event should invalidate results
    if (modelChangingEvents.includes(event.event_type) && get().isAnalyzed) {
      get().invalidateResults();
    }

    switch (event.event_type) {
      case 'model_updated':
        get().fetchModelState();
        break;
      case 'analysis_complete':
      case 'modal_analysis_complete':
      case 'nonlinear_analysis_complete':
        set({
          isAnalyzed: true,
          results: event.data.results as AnalysisResults,
          resultWarning: null,  // Clear warning on successful analysis
        });
        break;
      case 'error':
        set({ error: event.data.message as string });
        break;
      case 'tool_executed':
        // Refresh model state after tool execution
        get().fetchModelState();
        break;
      default:
        // For model-changing events, also refresh model state
        if (modelChangingEvents.includes(event.event_type)) {
          get().fetchModelState();
        }
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
