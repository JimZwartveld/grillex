import { useEffect } from 'react';
import Layout from './components/Layout';
import useStore from './stores/modelStore';

function App() {
  const { initializeSSE, disconnectSSE, fetchModelState } = useStore();

  useEffect(() => {
    // Initialize SSE connection and fetch initial model state
    initializeSSE();
    fetchModelState();

    // Cleanup on unmount
    return () => {
      disconnectSSE();
    };
  }, [initializeSSE, disconnectSSE, fetchModelState]);

  return <Layout />;
}

export default App;
