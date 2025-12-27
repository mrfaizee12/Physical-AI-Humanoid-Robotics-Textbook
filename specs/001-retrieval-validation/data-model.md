# Data Model: Retrieval Validation

## Entities

### Stored Vector
- **id**: Unique identifier for the vector in Qdrant
- **vector**: The embedding vector data (from Cohere)
- **payload**:
  - content: The text content that was embedded
  - url: Source URL of the content
  - chunk_index: Position of this chunk within the original document
  - source_title: Title of the source document
- **validation_status**: Status of integrity validation (valid, corrupted, missing)

### Validation Result
- **test_id**: Unique identifier for the validation test
- **timestamp**: When the validation was performed
- **vector_count**: Total number of vectors validated
- **success_count**: Number of vectors that passed validation
- **error_count**: Number of vectors that failed validation
- **integrity_score**: Percentage of vectors that passed integrity checks
- **retrieval_accuracy**: Accuracy metric for similarity search results
- **processing_time**: Time taken to complete validation

### Search Query
- **query_text**: The text used for similarity search
- **expected_results**: Expected relevant results for validation
- **actual_results**: Actual results returned by similarity search
- **precision_score**: Precision metric for the search results
- **confidence_threshold**: Minimum confidence required for valid matches

### Chunk Validation
- **chunk_id**: Identifier for the text chunk
- **original_content**: Original text content before embedding
- **retrieved_content**: Text content retrieved from Qdrant
- **url_match**: Whether the stored URL matches expected source
- **content_similarity**: Similarity score between original and retrieved content
- **validation_timestamp**: When validation was performed

### Retrieval Report
- **report_id**: Unique identifier for the report
- **validation_results**: Collection of validation results
- **sample_queries**: Test queries used for validation
- **sample_results**: Example search results for demonstration
- **performance_metrics**: Processing time, throughput, and other performance data
- **integrity_summary**: Summary of data integrity validation
- **recommendations**: Suggestions for improving retrieval quality

## Relationships

- **Validation Result** contains multiple **Stored Vector** validation statuses
- **Search Query** produces **Validation Result** with accuracy metrics
- **Chunk Validation** is part of overall **Validation Result**
- **Retrieval Report** aggregates multiple **Validation Result** entries