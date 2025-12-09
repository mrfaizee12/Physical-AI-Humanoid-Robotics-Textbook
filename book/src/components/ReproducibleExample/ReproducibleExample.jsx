import React from 'react';
import clsx from 'clsx';
import styles from './ReproducibleExample.module.css';

/**
 * Component for displaying reproducible examples
 * @param {Object} props - Component properties
 * @param {string} props.title - Title of the example
 * @param {string} props.description - Description of what the example demonstrates
 * @param {string} props.prerequisites - Prerequisites needed to run the example
 * @param {string} props.steps - Steps to reproduce the example
 * @param {string} props.expectedResult - What the user should expect to see
 * @param {string} [props.code] - Optional code snippet for the example
 * @param {string} [props.className] - Additional CSS classes
 */
export default function ReproducibleExample({
  title,
  description,
  prerequisites,
  steps,
  expectedResult,
  code,
  className
}) {
  return (
    <div className={clsx(styles.exampleContainer, className)}>
      <div className={styles.exampleHeader}>
        <h3 className={styles.exampleTitle}>{title}</h3>
      </div>

      <div className={styles.exampleSection}>
        <h4>Description</h4>
        <p>{description}</p>
      </div>

      <div className={styles.exampleSection}>
        <h4>Prerequisites</h4>
        <ul>
          {prerequisites.split('\n').map((item, index) => (
            <li key={index}>{item.trim()}</li>
          ))}
        </ul>
      </div>

      <div className={styles.exampleSection}>
        <h4>Steps</h4>
        <ol>
          {steps.split('\n').map((step, index) => (
            <li key={index}>{step.trim()}</li>
          ))}
        </ol>
      </div>

      <div className={styles.exampleSection}>
        <h4>Expected Result</h4>
        <p>{expectedResult}</p>
      </div>

      {code && (
        <div className={styles.exampleSection}>
          <h4>Code Example</h4>
          <pre className={styles.codeBlock}>
            <code>{code}</code>
          </pre>
        </div>
      )}
    </div>
  );
}