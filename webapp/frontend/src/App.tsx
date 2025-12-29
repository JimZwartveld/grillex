import { useEffect } from 'react';
import Layout from './components/Layout';
import useStore from './stores/modelStore';
import api from './api/client';

function App() {
  const { initializeSSE, disconnectSSE, fetchModelState } = useStore();

  useEffect(() => {
    // Initialize SSE connection
    initializeSSE();

    // Fetch initial model state, create default model if none exists
    const initializeModel = async () => {
      const response = await api.model.getState();
      if (response.success && response.data) {
        // If no model exists, create a default one
        if (!response.data.exists) {
          await api.model.create('New Model');
        }
      }
      // Fetch the model state (either existing or newly created)
      await fetchModelState();
    };

    initializeModel();

    // Cleanup on unmount
    return () => {
      disconnectSSE();
    };
  }, [initializeSSE, disconnectSSE, fetchModelState]);

  return <Layout />;
}

export default App;
