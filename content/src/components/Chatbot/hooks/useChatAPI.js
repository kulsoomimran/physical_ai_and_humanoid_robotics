import { useState } from 'react';

const useChatAPI = () => {
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);

  // Default configuration
  const defaultConfig = {
    timeout: 30000, // 30 seconds default
    retryAttempts: 3,
    backendUrl: 'https://kulsoomimran-rag-chatbot.hf.space' // Default backend URL
  };

  // Function to make a fetch request with timeout
  const fetchWithTimeout = async (url, options, timeout) => {
    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), timeout);

    try {
      const response = await fetch(url, {
        ...options,
        signal: controller.signal
      });
      clearTimeout(timeoutId);
      return response;
    } catch (err) {
      clearTimeout(timeoutId);
      if (err.name === 'AbortError') {
        throw new Error('Request timeout');
      }
      throw err;
    }
  };

  // Function to retry failed requests
  const retryRequest = async (requestFn, maxRetries) => {
    let lastError;

    for (let attempt = 0; attempt <= maxRetries; attempt++) {
      try {
        return await requestFn();
      } catch (error) {
        lastError = error;

        // If this was the last attempt, throw the error
        if (attempt === maxRetries) {
          throw lastError;
        }

        // Wait before retrying (exponential backoff)
        await new Promise(resolve => setTimeout(resolve, Math.pow(2, attempt) * 1000));
      }
    }

    throw lastError;
  };

  const sendQuestion = async (question, context = {}, config = {}) => {
    const finalConfig = { ...defaultConfig, ...config };

    setLoading(true);
    setError(null);

    try {
      const requestFn = async () => {
        const response = await fetchWithTimeout(
          `${finalConfig.backendUrl}/chat`,
          {
            method: 'POST',
            headers: {
              'Content-Type': 'application/json',
            },
            body: JSON.stringify({
              question: question,
              context: context
            }),
          },
          finalConfig.timeout
        );

        if (!response.ok) {
          const errorData = await response.json();
          throw new Error(errorData.message || `HTTP error! status: ${response.status}`);
        }

        return await response.json();
      };

      // Retry the request if it fails
      const data = await retryRequest(requestFn, finalConfig.retryAttempts);
      return data;
    } catch (err) {
      console.error('Error sending question to chat API:', err);
      setError(err.message);
      throw err;
    } finally {
      setLoading(false);
    }
  };

  // Function to send a streaming request (for future use if backend supports it)
  const sendStreamingQuestion = async (question, context = {}, onStream, config = {}) => {
    const finalConfig = { ...defaultConfig, ...config };

    setLoading(true);
    setError(null);

    try {
      const response = await fetchWithTimeout(
        `${finalConfig.backendUrl}/chat`,
        {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            question: question,
            context: context
          }),
        },
        finalConfig.timeout
      );

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.message || `HTTP error! status: ${response.status}`);
      }

      if (response.body) {
        const reader = response.body.getReader();
        const decoder = new TextDecoder();
        let buffer = '';

        try {
          while (true) {
            const { done, value } = await reader.read();
            if (done) break;

            buffer += decoder.decode(value, { stream: true });
            const lines = buffer.split('\n');
            buffer = lines.pop(); // Keep last incomplete line in buffer

            for (const line of lines) {
              if (line.trim() && line.startsWith('data: ')) {
                try {
                  const data = JSON.parse(line.substring(6)); // Remove 'data: ' prefix
                  if (data.answer) {
                    onStream(data.answer);
                  }
                } catch (e) {
                  // Skip malformed lines
                  console.warn('Could not parse streaming data:', line);
                }
              }
            }
          }
        } finally {
          reader.releaseLock();
        }
      } else {
        // Fallback to regular response if streaming is not supported
        const data = await response.json();
        onStream(data.answer);
      }
    } catch (err) {
      console.error('Error sending streaming question to chat API:', err);
      setError(err.message);
      throw err;
    } finally {
      setLoading(false);
    }
  };

  return {
    sendQuestion,
    sendStreamingQuestion,
    loading,
    error,
    config: defaultConfig
  };
};

export default useChatAPI;